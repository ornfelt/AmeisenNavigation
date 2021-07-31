#include "Main.hpp"

int main(int argc, const char* argv[])
{
#if defined(WIN32) || defined(WIN64)
    SetConsoleTitle(L"AmeisenNavigation Server"); 
#endif

    ChangeOutputColor(COLOR_WHITE);

    std::cout << "    ___                   _                 _   __           " << std::endl
              << "   /   |  ____ ___  ___  (_)_______  ____  / | / /___ __   __" << std::endl
              << "  / /| | / __ `__ \\/ _ \\/ / ___/ _ \\/ __ \\/  |/ / __ `/ | / /" << std::endl
              << " / ___ |/ / / / / /  __/ (__  )  __/ / / / /|  / /_/ /| |/ / " << std::endl
              << "/_/  |_/_/ /_/ /_/\\___/_/____/\\___/_/ /_/_/ |_/\\__,_/ |___/  " << std::endl
              << "                                        Server " << AMEISENNAV_VERSION << std::endl << std::endl;

    std::filesystem::path configPath(std::filesystem::path(argv[0]).parent_path().string() + "\\config.cfg");

    Config = new AmeisenNavConfig();

    if (argc > 1)
    {
        configPath = std::filesystem::path(argv[1]);

        if (!std::filesystem::exists(configPath))
        {
            ChangeOutputColor(COLOR_RED);
            std::cout << ">> Configfile does not exists: \"" << argv[1] << "\"" << std::endl;
            return 1;
        }
    }

    if (std::filesystem::exists(configPath))
    {
        ChangeOutputColor(COLOR_YELLOW);
        std::cout << ">> Loaded Configfile: \"" << configPath.string() << "\"" << std::endl;
        ChangeOutputColor(COLOR_WHITE);
        Config->Load(configPath);

        // directly save again to add new entries to it
        Config->Save(configPath);
    }
    else
    {
        std::cout << ">> Created default Configfile: \"" << configPath.string() << "\"" << std::endl;
        Config->Save(configPath);
        return 1;
    }

    // validate config
    if (!std::filesystem::exists(Config->mmapsPath))
    {
        ChangeOutputColor(COLOR_RED);
        std::cout << ">> MMAPS folder does not exists: \"" << Config->mmapsPath << "\"" << std::endl;
        return 1;
    }

    if (Config->maxPointPath <= 0)
    {
        ChangeOutputColor(COLOR_RED);
        std::cout << ">> iMaxPointPath has to be a value > 0" << std::endl;
        return 1;
    }

    if (Config->maxPolyPath <= 0)
    {
        ChangeOutputColor(COLOR_RED);
        std::cout << ">> iMaxPolyPath has to be a value > 0" << std::endl;
        return 1;
    }

    if (Config->port <= 0 || Config->port > 65535)
    {
        ChangeOutputColor(COLOR_RED);
        std::cout << ">> iMaxPolyPath has to be a value bewtween 1 and 65535" << std::endl;
        return 1;
    }

    // set ctrl+c handler to cleanup stuff when we exit
    if (!SetConsoleCtrlHandler(SigIntHandler, 1))
    {
        ChangeOutputColor(COLOR_RED);
        std::cout << ">> SetConsoleCtrlHandler() failed: " << GetLastError() << std::endl;
        return 1;
    }

    Nav = new AmeisenNavigation(Config->mmapsPath, Config->maxPolyPath, Config->maxPointPath);
    Server = new AnTcpServer(Config->ip, Config->port);

    Server->AddCallback(static_cast<char>(MessageType::PATH), PathCallback);
    Server->AddCallback(static_cast<char>(MessageType::RANDOM_POINT), RandomPointCallback);
    Server->AddCallback(static_cast<char>(MessageType::RANDOM_POINT_AROUND), RandomPointAroundCallback);
    Server->AddCallback(static_cast<char>(MessageType::MOVE_ALONG_SURFACE), MoveAlongSurfaceCallback);

    std::cout << ">> Starting server on: " << Config->ip << ":" << std::to_string(Config->port) << std::endl;
    Server->Run();

    std::cout << ">> Stopped server..." << std::endl;

    delete Config;
    delete Nav;
    delete Server;
}

int __stdcall SigIntHandler(unsigned long signal)
{
    if (signal == CTRL_C_EVENT || signal == CTRL_CLOSE_EVENT)
    {
        std::cout << ">> Received CTRL-C or CTRL-EXIT, stopping server..." << std::endl;
        Server->Stop();
    }

    return 1;
}

void PathCallback(ClientHandler* handler, char type, const void* data, int size)
{
    const PathRequestData request = *reinterpret_cast<const PathRequestData*>(data);

    int pathSize = 0;
    Vector3* path = new Vector3[Config->maxPointPath];

    if (Nav->GetPath(request.mapId, request.start, request.end, path, &pathSize))
    {
        if ((request.flags & static_cast<int>(PathRequestFlags::CATMULLROM)) && pathSize > 3)
        {
            std::vector<Vector3> output;
            SmoothPathCatmullRom(path, pathSize, &output, Config->catmullRomSplinePoints, Config->catmullRomSplineAlpha);
            handler->SendData(type, output.data(), sizeof(Vector3) * output.size());
        }
        else if ((request.flags & static_cast<int>(PathRequestFlags::CHAIKIN)) && pathSize > 2)
        {
            std::vector<Vector3> output;
            SmoothPathChaikinCurve(path, pathSize, &output);
            handler->SendData(type, output.data(), sizeof(Vector3) * output.size());
        }
        else
        {
            handler->SendData(type, path, sizeof(Vector3) * pathSize);
        }
    }
    else
    {
        handler->SendDataPtr(type, &Vector3Zero);
    }

    delete[] path;
}

void RandomPointCallback(ClientHandler* handler, char type, const void* data, int size)
{
    const int mapId = *reinterpret_cast<const int*>(data);
    Vector3 point;

    if (Nav->GetRandomPoint(mapId, &point))
    {
        handler->SendDataPtr(type, &point);
    }
    else
    {
        handler->SendDataPtr(type, &Vector3Zero);
    }
}

void RandomPointAroundCallback(ClientHandler* handler, char type, const void* data, int size)
{
    const RandomPointAroundData request = *reinterpret_cast<const RandomPointAroundData*>(data);
    Vector3 position;

    if (Nav->GetRandomPointAround(request.mapId, request.start, request.radius, &position))
    {
        handler->SendDataPtr(type, &position);
    }
    else
    {
        handler->SendDataPtr(type, &Vector3Zero);
    }
}

void MoveAlongSurfaceCallback(ClientHandler* handler, char type, const void* data, int size)
{
    const MoveRequestData request = *reinterpret_cast<const MoveRequestData*>(data);
    Vector3 position;

    if (Nav->MoveAlongSurface(request.mapId, request.start, request.end, &position))
    {
        handler->SendDataPtr(type, &position);
    }
    else
    {
        handler->SendDataPtr(type, &Vector3Zero);
    }
}