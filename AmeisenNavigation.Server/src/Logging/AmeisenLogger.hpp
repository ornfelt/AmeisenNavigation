#pragma once

#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#if defined WIN32 || defined WIN64
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>

constexpr auto CONSOLE_COLOR_BLUE = 3;
constexpr auto CONSOLE_COLOR_GRAY = 8;
constexpr auto CONSOLE_COLOR_GREEN = 10;
constexpr auto CONSOLE_COLOR_RED = 12;
constexpr auto CONSOLE_COLOR_WHITE = 15;
constexpr auto CONSOLE_COLOR_YELLOW = 14;
#endif

/**
 * @brief Macro for logging information messages.
 */
#define LogI(...) Log(__FUNCTION__, CONSOLE_COLOR_BLUE, CONSOLE_COLOR_WHITE, __VA_ARGS__)
 /**
  * @brief Macro for logging debug messages.
  */
#define LogD(...) Log(__FUNCTION__, CONSOLE_COLOR_GRAY, CONSOLE_COLOR_WHITE, __VA_ARGS__)
  /**
   * @brief Macro for logging warning messages.
   */
#define LogW(...) Log(__FUNCTION__, CONSOLE_COLOR_YELLOW, CONSOLE_COLOR_WHITE, __VA_ARGS__)
   /**
	* @brief Macro for logging error messages.
	*/
#define LogE(...) Log(__FUNCTION__, CONSOLE_COLOR_RED, CONSOLE_COLOR_WHITE, __VA_ARGS__)
	/**
	 * @brief Macro for logging success messages.
	 */
#define LogS(...) Log(__FUNCTION__, CONSOLE_COLOR_GREEN, CONSOLE_COLOR_WHITE, __VA_ARGS__)

	 /**
	  * @brief Flag indicating whether the console is opened.
	  */
static bool ConsoleOpened;
/**
 * @brief Handle to the standard output stream.
 */
static _iobuf* StdoutHandle;
/**
 * @brief Stream for logging messages to a file.
 */
static std::ofstream LogFileStream;
/**
 * @brief Delimiter used in the log messages.
 */
static const char* Delimiter = " >> ";

/**
 * @brief Logs a message to the console with optional color settings.
 *
 * @tparam Args The types of message arguments.
 * @param tag The log tag.
 * @param color The text color for the tag.
 * @param colorSecond The text color for the message.
 * @param args The message arguments.
 */
template<typename ...Args>
constexpr void Log(const std::string& tag, int color, int colorSecond, Args&& ...args)
{
#if defined WIN32 || defined WIN64
	static void* ConsoleHandle = GetStdHandle(STD_OUTPUT_HANDLE);
#endif

	std::cout << std::setw(14) << tag;

#if defined WIN32 || defined WIN64
	SetConsoleTextAttribute(ConsoleHandle, color);
#endif

	std::cout << Delimiter;

#if defined WIN32 || defined WIN64
	SetConsoleTextAttribute(ConsoleHandle, colorSecond);
#endif

	(std::cout << ... << args) << std::endl;
}