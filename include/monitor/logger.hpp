#ifndef _LOGGER__HPP_
#define _LOGGER__HPP_

#include <iostream>
#include <string>
#include <memory>
#include <stdexcept>

class logger{
public:
    logger(int verbalLevel, std::string header){
        _vL = verbalLevel;
        _header = "[" + header + "]: ";
    }

    // ~logger();

    template<typename ... Args>
    void log(std::ostream& os, const std::string& format, Args ... args){
        os << _header << _format(format, args...) << std::endl;
    }
    
    template<typename ... Args>
    void erro(const std::string& format, Args ... args){
        if (_vL > 0){
            std::cerr << tRd << _header << _format(format, args...) << tRs << std::endl;
        }
    }
    
    template<typename ... Args>
    void warn(const std::string& format, Args ... args){
        if (_vL > 1){
            std::cout << tYw << _header << _format(format, args...) << tRs << std::endl;
        }
    }
    
    template<typename ... Args>
    void prnt(const std::string& format, Args ... args){
        if (_vL > 2){
            std::cout << tGn << _header << _format(format, args...) << tRs << std::endl;
        }
    }

    template<typename ... Args>
    void debg(const std::string& format, Args ... args){
        if (_vL > 3){
            std::cout << tCn << _header << _format(format, args...) << tRs << std::endl;
        }
    }

private:
    int _vL;

    std::string _header;

    // ref:https://stackoverflow.com/questions/2342162/stdstring-formatting-like-sprintf
    template<typename ... Args>
    std::string _format(const std::string& format, Args ... args){
        int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
        if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
        auto size = static_cast<size_t>( size_s );
        std::unique_ptr<char[]> buf( new char[ size ] );
        std::snprintf( buf.get(), size, format.c_str(), args ... );
        return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
    }

    // set color
    // ref: https://stackoverflow.com/questions/2616906/how-do-i-output-coloured-text-to-a-linux-terminal
    std::string tRd = "\033[1;31m";
    std::string tGn = "\033[1;32m";
    std::string tYw = "\033[1;33m";
    std::string tCn = "\033[1;36m";
    std::string tRs = "\033[0m";
};

#endif