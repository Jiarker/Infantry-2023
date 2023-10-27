#ifndef MODULE_HPP_
#define MODULE_HPP_

#include "Common.hpp"

#include <opencv4/opencv2/opencv.hpp>

#include <map>
#include <string>
#include <functional>
#include <memory>


class Interpreter
{
public:

    virtual bool init(std::vector<void*> inputs_address, std::string params_file, std::string params_position) = 0;

    virtual std::string metadata() = 0;

    virtual void process() = 0;

    virtual std::vector<void*> outputs() = 0;

};


class PreProcesser
{

public:
    
    virtual bool init(
            cv::Size camera_size, cv::Size net_input_size, 
            double normalize_scale = 255.0, int normalize_bias = 0,
            bool bind_uesr_tensor_addr = false, void* tensor_addr = nullptr, int tensor_type = CV_32FC3
        ) = 0;
    
    virtual bool init_with_log(
            cv::Size camera_size, cv::Size net_input_size, 
            double normalize_scale = 255.0, int normalize_bias = 0,
            bool bind_uesr_tensor_addr = false, void* tensor_addr = nullptr, int tensor_type = CV_32FC3
        ) = 0;

    virtual bool init_with_yaml(std::string params_file, std::string params_position) = 0;


    virtual std::string metadata() = 0;

    virtual std::string error_messages() = 0;

    virtual cv::Mat preprocessing_img() = 0;

    virtual void process(cv::Mat & image) = 0;

    virtual std::vector<void*> outputs() = 0;
    
};


class PostProcesser
{
public:
    virtual bool init(void* tensor_addr, std::string params_file, std::string params_position) = 0;

    virtual std::string metadata() = 0;

    virtual void process(DetectResultList & output) = 0;
    virtual void process(SegmentResultList & output) = 0;

};



class InterpreterFactory
{
public:
    template<typename T>
    struct register_t
    {
        register_t(const std::string& key)
        {
            InterpreterFactory::get().map_.emplace(key, [] { return new T(); });
        }

        template<typename... Args>
        register_t(const std::string& key, Args... args)
        {
            InterpreterFactory::get().map_.emplace(key, [&] { return new T(args...); });   
        }
    };

    static Interpreter* produce(const std::string& key)
    {
        if (map_.find(key) == map_.end())
            throw std::invalid_argument("the prep key is not exist!");
        return map_[key]();
    }

    static std::unique_ptr<Interpreter> produce_unique(const std::string& key)
    {
        return std::unique_ptr<Interpreter>(produce(key));
    }

    static std::shared_ptr<Interpreter> produce_shared(const std::string& key)
    {
        return std::shared_ptr<Interpreter>(produce(key));
    }

private:
    InterpreterFactory() {};
    InterpreterFactory(const InterpreterFactory&) = delete;
    InterpreterFactory(InterpreterFactory&&) = delete;

    static InterpreterFactory& get()
    {
        static InterpreterFactory instance;
        return instance;
    }

    static std::map<std::string, std::function<Interpreter*()>> map_;
};

#define REGISTER_INTERPRETER_VNAME(T) reg_msg_##T##_
#define REGISTER_INTERPRETER(T, key, ...) static InterpreterFactory::register_t<T> REGISTER_INTERPRETER_VNAME(T)(key, ##__VA_ARGS__);


class PreProcesserFactory
{
public:
    template<typename T>
    struct register_t
    {
        register_t(const std::string& key)
        {
            PreProcesserFactory::get().map_.emplace(key, [] { return new T(); });
        }

        template<typename... Args>
        register_t(const std::string& key, Args... args)
        {
            PreProcesserFactory::get().map_.emplace(key, [&] { return new T(args...); });   
        }
    };

    static PreProcesser* produce(const std::string& key)
    {
        if (map_.find(key) == map_.end())
            throw std::invalid_argument("the prep key is not exist!");
        return map_[key]();
    }

    static std::unique_ptr<PreProcesser> produce_unique(const std::string& key)
    {
        return std::unique_ptr<PreProcesser>(produce(key));
    }

    static std::shared_ptr<PreProcesser> produce_shared(const std::string& key)
    {
        return std::shared_ptr<PreProcesser>(produce(key));
    }

private:
    PreProcesserFactory() {};
    PreProcesserFactory(const PreProcesserFactory&) = delete;
    PreProcesserFactory(PreProcesserFactory&&) = delete;

    static PreProcesserFactory& get()
    {
        static PreProcesserFactory instance;
        return instance;
    }

    static std::map<std::string, std::function<PreProcesser*()>> map_;
};

#define REGISTER_PREPROCESSER_VNAME(T) reg_msg_##T##_
#define REGISTER_PREPROCESSER(T, key, ...) static PreProcesserFactory::register_t<T> REGISTER_PREPROCESSER_VNAME(T)(key, ##__VA_ARGS__);



class PostProcesserFactory
{
public:
    template<typename T>
    struct register_t
    {
        register_t(const std::string& key)
        {
            PostProcesserFactory::get().map_.emplace(key, [] { return new T(); });
        }

        template<typename... Args>
        register_t(const std::string& key, Args... args)
        {
            PostProcesserFactory::get().map_.emplace(key, [&] { return new T(args...); });   
        }
    };

    static PostProcesser* produce(const std::string& key)
    {
        if (map_.find(key) == map_.end())
            throw std::invalid_argument("the posp key is not exist!");
        return map_[key]();
    }

    static std::unique_ptr<PostProcesser> produce_unique(const std::string& key)
    {
        return std::unique_ptr<PostProcesser>(produce(key));
    }

    static std::shared_ptr<PostProcesser> produce_shared(const std::string& key)
    {
        return std::shared_ptr<PostProcesser>(produce(key));
    }

private:
    PostProcesserFactory() {};
    PostProcesserFactory(const PostProcesserFactory&) = delete;
    PostProcesserFactory(PostProcesserFactory&&) = delete;

    static PostProcesserFactory& get()
    {
        static PostProcesserFactory instance;
        return instance;
    }

    static std::map<std::string, std::function<PostProcesser*()>> map_;
};

#define REGISTER_POSTPROCESSER_VNAME(T) reg_msg_##T##_
#define REGISTER_POSTPROCESSER(T, key, ...) static PostProcesserFactory::register_t<T> REGISTER_POSTPROCESSER_VNAME(T)(key, ##__VA_ARGS__);






#endif
