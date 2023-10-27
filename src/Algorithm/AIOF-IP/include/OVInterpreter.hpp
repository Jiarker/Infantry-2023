#ifndef OVINTERPRETER_HPP_
#define OVINTERPRETER_HPP_

#include "Module.hpp"
#include "openvino/openvino.hpp"

class OVInterpreter : public Interpreter
{
public:
    bool init(std::vector<void*> inputs_address, std::string params_file, std::string params_position) override
    {
        cv::FileStorage fs(params_file, cv::FileStorage::READ);
        fs["interpreters"][params_position]["params"]["model_path"] >> model_path_;
        fs["interpreters"][params_position]["params"]["device_name"] >> device_name_;

        ov::Core core;
        model_ = core.read_model(model_path_);

        compiled_model_ = core.compile_model(model_, device_name_, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));

        infer_request_ = compiled_model_.create_infer_request();


        for (size_t i = 0; i < inputs_address.size(); i++)
        {
            auto input_port = compiled_model_.input(i);
            input_tensor_ = ov::Tensor(
                    input_port.get_element_type(), 
                    input_port.get_shape(), 
                    inputs_address[i]
                );

            infer_request_.set_input_tensor(input_tensor_);

        }

        // std::cout << metadata() << std::endl;
        
        return true;
    }

    std::string metadata() override
    {
        std::stringstream metadata_stream;

        metadata_stream
                << "INFO: (Fun) print_metadata() =>"                            << "\n" 
                << "================= interpreter metadata ================="   << "\n" 
                << "model name: " << model_->get_friendly_name()                << "\n";
        
        std::vector<ov::Output<ov::Node>> inputs = model_->inputs();

        size_t inputs_count = 0;
        for (ov::Output<ov::Node> input : inputs)
        {
            metadata_stream
                << "    input" << std::to_string(inputs_count) << ":"     << "\n";
            
            const std::string name = input.get_names().empty() ? "NONE" : input.get_any_name();
            metadata_stream
                << "        name: "       << name                         << "\n";

            const ov::element::Type type = input.get_element_type();
            metadata_stream
                << "        type: "       << type                         << "\n";

            const ov::Shape shape = input.get_shape();
            metadata_stream
                << "        shape: "      << shape                        << "\n";
            metadata_stream
                << "        address: "    << infer_request_.get_input_tensor(inputs_count).data()   << "\n";
        }

        std::vector<ov::Output<ov::Node>> outputs = model_->outputs();

        size_t outputs_count = 0;
        for (ov::Output<ov::Node> output : outputs)
        {
            metadata_stream
                << "    output" << std::to_string(outputs_count) << ":"         << "\n";
            
            const std::string name = output.get_names().empty() ? "NONE" : output.get_any_name();
            metadata_stream
                << "        name: "      << name                         << "\n";

            const ov::element::Type type = output.get_element_type();
            metadata_stream
                << "        type: "      << type                         << "\n";

            const ov::Shape shape = output.get_shape();
            metadata_stream
                << "        shape: "     << shape                        << "\n";
            metadata_stream
                << "        address: "    << infer_request_.get_output_tensor(outputs_count).data()   << "\n";
        }

        metadata_stream
            << "========================================================"       << "\n";

        return metadata_stream.str();
    }

    void process() override
    {
        // auto mat = cv::Mat(
        //             640,
        //             640, 
        //             CV_32FC3, 
        //             input_tensor_.data()
        //         );
        // cv::imshow("inter", mat);

        infer_request_.infer();


        // auto output_data = infer_request_.get_output_tensor().data<float>();
        // for (size_t i = 0; i < 25200*51; i+=51)
        // {
        //     // bbox conf   
        //     if (output_data[i + 4] > 0.1) { std::cout << "fine: " << output_data[i + 4] << std::endl; }         
        //     // if (p_tensor_[i + 4] < conf_thre_) { continue; }
        // }
    }

    std::vector<void*> outputs() override
    {
        std::vector<void*> outputs;
        for (size_t i = 0; i < model_->outputs().size(); i++)
        {
            outputs.push_back(reinterpret_cast<void*>(infer_request_.get_output_tensor(i).data<float>()));
        }
        return outputs;
    }
    
    OVInterpreter() {}
    ~OVInterpreter() {}


    // parameter
    std::string model_path_;
    std::string device_name_;

    std::shared_ptr<ov::Model> model_;
    ov::CompiledModel compiled_model_;

    ov::Tensor input_tensor_;
    ov::Tensor output_tensor_;

    ov::InferRequest infer_request_;

    // for debug
    std::string error_message_;
};

#endif