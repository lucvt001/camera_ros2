#include <camera_wrapper/utils.h>

std::string generateMP4FileName() {
    // Get the current time
    std::time_t now = std::time(nullptr);
    std::tm* now_tm = std::localtime(&now);

    // Create a string stream to format the date and time
    std::ostringstream oss;
    oss << std::put_time(now_tm, "%Y-%m-%d-%H-%M-%S") << ".mp4";

    // Return the formatted string
    return oss.str();
}