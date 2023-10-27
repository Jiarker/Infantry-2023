#include "../include/Module.hpp"

std::map<std::string, std::function<Interpreter*()>> InterpreterFactory::map_;
std::map<std::string, std::function<PreProcesser*()>> PreProcesserFactory::map_;
std::map<std::string, std::function<PostProcesser*()>> PostProcesserFactory::map_;

