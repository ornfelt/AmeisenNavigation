#include "Main.hpp"

/**
 * @brief The main entry point of the AmeisenNavigation server.
 *
 * This function initializes and runs the AmeisenNavigation server. It reads configuration settings,
 * handles signal interruptions (Ctrl+C), sets up the server, and starts listening for incoming connections.
 *
 * @param argc The number of command-line arguments.
 * @param argv An array of pointers to command-line arguments.
 * @return Returns 0 on success or 1 on error.
 */
int main(int argc, const char* argv[])
{
#if defined(WIN32) || defined(WIN64)
	SetConsoleTitle(L"AmeisenNavigation Server");
	SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 15);
#endif

	std::cout << "      ___                   _                 _   __           " << std::endl
		<< "     /   |  ____ ___  ___  (_)_______  ____  / | / /___ __   __" << std::endl
		<< "    / /| | / __ `__ \\/ _ \\/ / ___/ _ \\/ __ \\/  |/ / __ `/ | / /" << std::endl
		<< "   / ___ |/ / / / / /  __/ (__  )  __/ / / / /|  / /_/ /| |/ / " << std::endl
		<< "  /_/  |_/_/ /_/ /_/\\___/_/____/\\___/_/ /_/_/ |_/\\__,_/ |___/  " << std::endl
		<< "                                          Server " << AMEISENNAV_VERSION << std::endl << std::endl;

	std::filesystem::path configPath(std::filesystem::path(argv[0]).parent_path().string() + "\\config.cfg");
	Config = new AmeisenNavConfig();

	if (argc > 1)
	{
		configPath = std::filesystem::path(argv[1]);

		if (!std::filesystem::exists(configPath))
		{
			LogE("Configfile does not exists: \"", argv[1], "\"");
			std::cin.get();
			return 1;
		}
	}

	if (std::filesystem::exists(configPath))
	{
		Config->Load(configPath);
		LogI("Loaded Configfile: \"", configPath.string(), "\"");

		// directly save again to add new entries to it
		Config->Save(configPath);
	}
	else
	{
		Config->Save(configPath);

		LogI("Created default Configfile: \"", configPath.string(), "\"");
		LogI("Edit it and restart the server, press any key to exit...");
		std::cin.get();
		return 1;
	}

	// validate config
	if (!std::filesystem::exists(Config->mmapsPath))
	{
		LogE("MMAPS folder does not exists: \"", Config->mmapsPath, "\"");
		std::cin.get();
		return 1;
	}

	if (Config->maxPolyPath <= 0)
	{
		LogE("iMaxPolyPath has to be a value > 0");
		std::cin.get();
		return 1;
	}

	if (Config->port <= 0 || Config->port > 65535)
	{
		LogE("iPort has to be a value bewtween 1 and 65535");
		std::cin.get();
		return 1;
	}

	if (Config->maxSearchNodes <= 0 || Config->maxSearchNodes > 65535)
	{
		LogE("iMaxSearchNodes has to be a value bewtween 1 and 65535");
		std::cin.get();
		return 1;
	}

	// set ctrl+c handler to cleanup stuff when we exit
	if (!SetConsoleCtrlHandler(SigIntHandler, 1))
	{
		LogE("SetConsoleCtrlHandler() failed: ", GetLastError());
		std::cin.get();
		return 1;
	}

	Nav = new AmeisenNavigation(Config->mmapsPath, Config->maxPolyPath, Config->maxSearchNodes);
	Server = new AnTcpServer(Config->ip, Config->port);

	Server->SetOnClientConnected(OnClientConnect);
	Server->SetOnClientDisconnected(OnClientDisconnect);

	Server->AddCallback(static_cast<char>(MessageType::PATH), PathCallback);
	Server->AddCallback(static_cast<char>(MessageType::RANDOM_PATH), RandomPathCallback);
	Server->AddCallback(static_cast<char>(MessageType::RANDOM_POINT), RandomPointCallback);
	Server->AddCallback(static_cast<char>(MessageType::RANDOM_POINT_AROUND), RandomPointAroundCallback);
	Server->AddCallback(static_cast<char>(MessageType::MOVE_ALONG_SURFACE), MoveAlongSurfaceCallback);
	Server->AddCallback(static_cast<char>(MessageType::CAST_RAY), CastRayCallback);

	LogS("Starting server on: ", Config->ip, ":", std::to_string(Config->port));
	Server->Run();

	LogI("Stopped server...");
	delete Config;
	delete Nav;
	delete Server;

	for (const auto& kv : ClientPathBuffers)
	{
		if (kv.second.first)
		{
			delete[] kv.second.first;
		}

		if (kv.second.second)
		{
			delete[] kv.second.second;
		}
	}
}

/**
 * @brief Signal handler for handling SIGINT events.
 *
 * This function is used to handle the SIGINT signal, specifically CTRL-C or CTRL-CLOSE events.
 *
 * @param signal The signal code.
 * @return Returns 1.
 */
int __stdcall SigIntHandler(unsigned long signal)
{
	if (signal == CTRL_C_EVENT || signal == CTRL_CLOSE_EVENT)
	{
		LogI("Received CTRL-C or CTRL-EXIT, stopping server...");
		Server->Stop();
	}

	return 1;
}

/**
 * @brief Function called when a client connects.
 *
 * This function is called when a client connects to the server.
 *
 * @param handler The ClientHandler object for the connected client.
 */
void OnClientConnect(ClientHandler* handler) noexcept
{
	LogI("Client Connected: ", handler->GetIpAddress(), ":", handler->GetPort());

	ClientPathBuffers[handler->GetId()] = std::make_pair(new float[Config->maxPolyPath * 3], new float[Config->maxPolyPath * 3]);
	Nav->NewClient(handler->GetId(), static_cast<CLIENT_VERSION>(Config->clientVersion));
}

/**
 * @brief Function called when a client disconnects.
 *
 * This function is called when a client disconnects from the server.
 *
 * @param handler The ClientHandler object for the disconnected client.
 */
void OnClientDisconnect(ClientHandler* handler) noexcept
{
	Nav->FreeClient(handler->GetId());

	delete[] ClientPathBuffers[handler->GetId()].first;
	ClientPathBuffers[handler->GetId()].first = nullptr;

	delete[] ClientPathBuffers[handler->GetId()].second;
	ClientPathBuffers[handler->GetId()].second = nullptr;

	LogI("Client Disconnected: ", handler->GetIpAddress(), ":", handler->GetPort());
}

/**
 * @brief Callback function for handling path data.
 *
 * This function is a callback for handling path data and delegates to GenericPathCallback with PathType::STRAIGHT.
 *
 * @param handler The ClientHandler object.
 * @param type The data type.
 * @param data Pointer to the data.
 * @param size The size of the data.
 */
void PathCallback(ClientHandler* handler, char type, const void* data, int size) noexcept
{
	GenericPathCallback(handler, type, data, size, PathType::STRAIGHT);
}

/**
 * @brief Callback function for handling random path data.
 *
 * This function is a callback for handling random path data and delegates to GenericPathCallback with PathType::RANDOM.
 *
 * @param handler The ClientHandler object.
 * @param type The data type.
 * @param data Pointer to the data.
 * @param size The size of the data.
 */
void RandomPathCallback(ClientHandler* handler, char type, const void* data, int size) noexcept
{
	GenericPathCallback(handler, type, data, size, PathType::RANDOM);
}

/**
 * @brief Callback function for generating and sending a random point to the client.
 *
 * This function generates a random point on or around the specified map and sends it to the client.
 *
 * @param handler The ClientHandler object.
 * @param type The data type.
 * @param data Pointer to the data (contains the map ID).
 * @param size The size of the data (not used in this function).
 */
void RandomPointCallback(ClientHandler* handler, char type, const void* data, int size) noexcept
{
	const int mapId = *reinterpret_cast<const int*>(data);
	float point[3]{};

	Nav->GetRandomPoint(handler->GetId(), mapId, point);
	handler->SendData(type, point, VEC3_SIZE);
}

/**
 * @brief Callback function for generating and sending a random point around a specified location to the client.
 *
 * This function generates a random point around a specified location on the specified map and sends it to the client.
 *
 * @param handler The ClientHandler object.
 * @param type The data type.
 * @param data Pointer to the data (contains request details like map ID, start location, and radius).
 * @param size The size of the data (not used in this function).
 */
void RandomPointAroundCallback(ClientHandler* handler, char type, const void* data, int size) noexcept
{
	const RandomPointAroundData request = *reinterpret_cast<const RandomPointAroundData*>(data);
	float point[3]{};

	Nav->GetRandomPointAround(handler->GetId(), request.mapId, request.start, request.radius, point);
	handler->SendData(type, point, VEC3_SIZE);
}

/**
 * @brief Callback function for moving along a surface and sending the result to the client.
 *
 * This function computes a path for moving along a surface between two points on the specified map and sends it to the client.
 *
 * @param handler The ClientHandler object.
 * @param type The data type.
 * @param data Pointer to the data (contains request details like map ID, start, and end points).
 * @param size The size of the data (not used in this function).
 */
void MoveAlongSurfaceCallback(ClientHandler* handler, char type, const void* data, int size) noexcept
{
	const MoveRequestData request = *reinterpret_cast<const MoveRequestData*>(data);
	float point[3]{};

	Nav->MoveAlongSurface(handler->GetId(), request.mapId, request.start, request.end, point);
	handler->SendData(type, point, VEC3_SIZE);
}

/**
 * @brief Callback function for casting a ray and sending the result to the client.
 *
 * This function casts a ray from the start point to the end point on the specified map and sends the result to the client.
 *
 * @param handler The ClientHandler object.
 * @param type The data type.
 * @param data Pointer to the data (contains request details like map ID and ray endpoints).
 * @param size The size of the data (not used in this function).
 */
void CastRayCallback(ClientHandler* handler, char type, const void* data, int size) noexcept
{
	const CastRayData request = *reinterpret_cast<const CastRayData*>(data);
	dtRaycastHit hit;

	if (Nav->CastMovementRay(handler->GetId(), request.mapId, request.start, request.end, &hit))
	{
		handler->SendData(type, request.end, VEC3_SIZE);
	}
	else
	{
		float zero[3]{};
		handler->SendData(type, zero, VEC3_SIZE);
	}
}

/**
 * @brief Generic callback function for handling path-related requests and sending the result to the client.
 *
 * This function handles path-related requests from clients, including generating paths and smoothing them based on the specified path type.
 *
 * @param handler The ClientHandler object.
 * @param type The data type.
 * @param data Pointer to the data (contains request details like map ID, start, and end points, and path flags).
 * @param size The size of the data (not used in this function).
 * @param pathType The type of path to generate (STRAIGHT or RANDOM).
 */
void GenericPathCallback(ClientHandler* handler, char type, const void* data, int size, PathType pathType) noexcept
{
	const PathRequestData request = *reinterpret_cast<const PathRequestData*>(data);

	int pathSize = 0;
	float* pathBuffer = ClientPathBuffers[handler->GetId()].first;

	bool pathGenerated = false;

	switch (pathType)
	{
	case PathType::STRAIGHT:
		pathGenerated = Nav->GetPath(handler->GetId(), request.mapId, request.start, request.end, pathBuffer, &pathSize);
		break;
	case PathType::RANDOM:
		pathGenerated = Nav->GetRandomPath(handler->GetId(), request.mapId, request.start, request.end, pathBuffer, &pathSize, Config->randomPathMaxDistance);
		break;
	}

	if (pathGenerated)
	{
		if ((request.flags & static_cast<int>(PathRequestFlags::CATMULLROM)) && pathSize > 9)
		{
			int smoothedPathSize = 0;
			float* smoothedPathBuffer = ClientPathBuffers[handler->GetId()].second;
			Nav->SmoothPathCatmullRom(pathBuffer, pathSize, smoothedPathBuffer, &smoothedPathSize, Config->catmullRomSplinePoints, Config->catmullRomSplineAlpha);

			handler->SendData(type, smoothedPathBuffer, smoothedPathSize * sizeof(float));
		}
		else if ((request.flags & static_cast<int>(PathRequestFlags::CHAIKIN)) && pathSize > 6)
		{
			int smoothedPathSize = 0;
			float* smoothedPathBuffer = ClientPathBuffers[handler->GetId()].second;
			Nav->SmoothPathChaikinCurve(pathBuffer, pathSize, smoothedPathBuffer, &smoothedPathSize);

			handler->SendData(type, smoothedPathBuffer, smoothedPathSize * sizeof(float));
		}
		else
		{
			handler->SendData(type, pathBuffer, pathSize * sizeof(float));
		}
	}
	else
	{
		float zero[3]{};
		handler->SendData(type, zero, VEC3_SIZE);
	}
}