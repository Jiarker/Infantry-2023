#ifndef RUNTIME_HPP_
#define RUNTIME_HPP_

#include "ModuleRegister.hpp"

class Runtime
{
public:
    bool init(std::string yaml_file)
    {
        ModuleRegister module_register;
        module_register();

        std::cout << yaml_file << std::endl;

        cv::FileStorage fs(yaml_file, cv::FileStorage::READ);


        if (fs.isOpened()) {
            init_msgs_ << "open success : " << yaml_file << std::endl;
        }
        else {
            init_msgs_ << "open error : " << yaml_file << std::endl;
            return false;
        }

        init_msgs_ << "Pre-Processers Init ... " << std::endl;
        // preprocesser
        int preps_num;
        fs["pre_processers"]["num"] >> preps_num;
        for (size_t i = 0; i < preps_num; i++)
        {
            std::string prep_tag = "pre_processer" + std::to_string(i);
            std::string prep_name;
            fs["pre_processers"][prep_tag]["name"] >> prep_name;

            init_msgs_ << prep_name << std::endl;

            std::shared_ptr<PreProcesser> prep = PreProcesserFactory::produce_shared(prep_name);

            if (prep->init_with_yaml(yaml_file, prep_tag) == true) {
                init_msgs_ << "prep" << i << " init success" << std::endl;
                init_msgs_ << prep->metadata() << std::endl;
            }
            else {
                init_msgs_ << "prep" << i << " init fault" << std::endl;
                init_msgs_ << prep->metadata() << std::endl;
                return false;
            }
            preps_.push_back(prep);
        }

        init_msgs_ << "Interpreters Init ... " << std::endl;

        int inters_num;
        fs["interpreters"]["num"] >> inters_num;
        for (size_t i = 0; i < inters_num; i++)
        {

            std::string inter_tag = "interpreter" + std::to_string(i);
            std::string inter_name;
            fs["interpreters"][inter_tag]["name"] >> inter_name;

            std::shared_ptr<Interpreter> inter = InterpreterFactory::produce_shared(inter_name);

            std::vector<void*> inputs_addr;
            
            int connects_num;
            fs["interpreters"][inter_tag]["connects"]["num"] >> connects_num;
            for (size_t j = 0; j < connects_num; j++)
            {
                int connect_id, connect_port;
                fs["interpreters"][inter_tag]["connects"]["connect"+std::to_string(j)]["id"] >> connect_id;
                fs["interpreters"][inter_tag]["connects"]["connect"+std::to_string(j)]["port"] >> connect_port;
                inputs_addr.push_back(preps_[connect_id]->outputs()[connect_port]);
            }
            
            if (inter->init(inputs_addr, yaml_file, inter_tag) == true) {
                init_msgs_ << "infer" << i << " init success" << std::endl;
                init_msgs_ << inter->metadata() << std::endl;
            }
            else {
                init_msgs_ << "infer" << i << " init fault" << std::endl;
                init_msgs_ << inter->metadata() << std::endl;
                return false;
            }
            inters_.push_back(inter);
        }

        init_msgs_ << "Post-Processers Init ... " << std::endl;

        int posps_num;
        fs["post_processers"]["num"] >> posps_num;
        for (size_t i = 0; i < posps_num; i++)
        {

            std::string posp_tag = "post_processer" + std::to_string(i);
            std::string posp_name;
            fs["post_processers"][posp_tag]["name"] >> posp_name;
            std::shared_ptr<PostProcesser> posp = PostProcesserFactory::produce_shared(posp_name);
            
            int connect_id, connect_port;
            fs["post_processers"][posp_tag]["connect"]["id"] >> connect_id;
            fs["post_processers"][posp_tag]["connect"]["port"] >> connect_port;
            
            if (posp->init(inters_[connect_id]->outputs()[connect_port], yaml_file, posp_tag) == true) {
                init_msgs_ << "posp" << i << " init success" << std::endl;
                init_msgs_ << posp->metadata() << std::endl;
            }
            else {
                init_msgs_ << "posp" << i << " init fault" << std::endl;
                init_msgs_ << posp->metadata() << std::endl;
            }
            posps_.push_back(posp);
        }

        init_msgs_ << "Lodding tags labble ... " << std::endl;
        init_msgs_ << "labbles : " << std::endl << "[";
        cv::FileNode tags_fn = fs["result_tags"];
        // if(tensor_shape_vector_fn.type() != cv::FileNode::SEQ) {}
        for (auto it = tags_fn.begin(); it != tags_fn.end(); it++) {          
            cls_names.push_back((std::string)*it);
            init_msgs_ << (std::string)*it << " ";
        }
        init_msgs_ << "]" << std::endl;

        init_msgs_ << "Finish Init" << std::endl;

        return true;
    }

    // TODO: MultiThread Runtime Excutor
    template<typename T>
    void run(std::vector<cv::Mat> & mats, std::vector<T> & outputs)
    {
        // std::cout << "STEP: preps" << std::endl;

        auto t1 = std::chrono::steady_clock::now();

        size_t counter = 0;
        for (auto prep : preps_)
        {
            prep->process(mats[counter]);
            // cv::imshow("prop img", prep->preprocessing_img());
            counter++;
        }

        // std::cout << "STEP: inters" << std::endl;

        auto t2 = std::chrono::steady_clock::now();

        for (auto inter : inters_) {
            inter->process();
        }

        // std::cout << "STEP: posps" << std::endl;

        auto t3 = std::chrono::steady_clock::now();

        for (auto posp : posps_)
        {
            T output;
            posp->process(output);
            outputs.push_back(output);
        }

        auto t4 = std::chrono::steady_clock::now();

        preps_cost_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
        infers_cost_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();
        posps_cost_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count();

    }


    template<typename T>
    void result_mats_gen(std::vector<cv::Mat> & src_mats, std::vector<T> & results, std::vector<cv::Mat> & dst_mats)
    {
        for (auto result : results) 
        {
            if ((typeid(T) == typeid(DetectResultList)) 
                && (preps_.size() == 1) 
                && (posps_.size() == 1)
                && (src_mats.size() == 1) 
            ) {
                // cv::Mat result_img = preps_[0]->preprocessing_img();
                cv::Mat result_img = src_mats[0];
                for (auto result_info : result) {
                    cv::rectangle(result_img, result_info.bbox, cv::Scalar(0, 255, 0), 2);
                    cv::putText(result_img, cls_names[result_info.id], result_info.bbox.tl(), 1, 1, cv::Scalar(0,255,200));
                    for(int i = 0; i < result_info.points.size(); i++) {
                        cv::circle(result_img, result_info.points[i], 2, cv::Scalar(2,0,255), -1);
                    }
                }
                dst_mats.push_back(result_img);
            }
        }
    }

    std::string init_messages() { return init_msgs_.str(); }
    int64_t preps_cost_time() { return preps_cost_time_; }
    int64_t infers_cost_time() { return infers_cost_time_; }
    int64_t posps_cost_time() { return posps_cost_time_; }

private:
    std::vector<std::shared_ptr<PreProcesser>> preps_;
    std::vector<std::shared_ptr<Interpreter>> inters_;
    std::vector<std::shared_ptr<PostProcesser>> posps_;

    std::vector<std::string> cls_names;

    std::stringstream init_msgs_;
    
    int64_t preps_cost_time_;
    int64_t infers_cost_time_;
    int64_t posps_cost_time_;

};

#endif