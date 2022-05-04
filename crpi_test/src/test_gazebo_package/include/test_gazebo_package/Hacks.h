#pragma once

#include <boost/filesystem.hpp>
#include <ros/package.h>

#if 0
namespace tf2
{
	/** TO HANDLE THIS ERROR! IN tf2/convert.h
			template <>
		template <typename A, typename B>
		inline void Converter<true, false>::convert(const A& a, B& b)
		{
		#ifdef _MSC_VER
		  tf2::fromMsg(a, b);
		#else
		  fromMsg(a, b);
		#endif
		}*/
	template<typename A, typename B>
	void fromMsg(const A&, B& b) {}

	template<typename A, typename B>
	void toMsg(const A&, B& b) {}
}
#endif
#include <ros/ros.h>
#include <tf/tf.h>

template <typename... Args>
std::string format(const std::string &format, Args... args)
{
	size_t size = snprintf(nullptr, 0, format.c_str(), args...) + 1; // Extra space for '\0'
	std::unique_ptr<char[]> buf(new char[size]);
	snprintf(buf.get(), size, format.c_str(), args...);
	return std::string(buf.get(), buf.get() + size - 1); // We don't want the '\0' inside
}

/**
 * @brief vectorDump to a dump of a vector to a stringstream, return string.
 * @param v vector of template type T
 * @param separator separator between type ostream.
 * @return std string of stringstream
 */
template<typename T>
inline std::string vectorDump(std::vector<T> v, std::string separator=",") {
    std::stringstream s;

    for (size_t i = 0; i < v.size(); i++) {
        if(i>0)
            s << separator;
        s << v[i] ;
    }
    return s.str();
}
inline std::string getExePath(std::string filepath)
{
	if (filepath[0] == 0)
		return "C:\\opt\\ros\\noetic\\logfile.txt";
	std::cout << filepath << "\n";
	char sep = '\\'; //boost::filesystem::path::separator;
	std::string path= filepath.substr(0, filepath.find_last_of(sep) + 1);
	std::cout << path << "\n";
	return path;
}
inline std::string getDataTimeFilestamp(std::string pre)
{
	time_t currentTime = time(0);
	tm *currentDate = localtime(&currentTime);

	return format("%s%d-%d-%d-%d.%d.%d",
				  pre.c_str(),
				  currentDate->tm_year + 1900, currentDate->tm_mon + 1, currentDate->tm_mday,
				  currentDate->tm_hour, currentDate->tm_min, currentDate->tm_sec);
}
inline std::string getRosLogTimeFilestamp(std::string pkg, std::string pre)
{
	// rosrun pkg node
	std::string path = ros::package::getPath(pkg);
	std::replace(path.begin(), path.end(), '/', '\\');
	path += "\\..\\..\\logs\\" + getDataTimeFilestamp(pre) + ".dat";
	return path;
}

