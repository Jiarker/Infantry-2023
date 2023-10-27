#ifndef RM_V5_POST_PEOCESSER_HPP_
#define RM_V5_POST_PEOCESSER_HPP_

#include "Module.hpp"

class RmV5PostProcesser : public PostProcesser
{

public:
    bool init(void* tensor_addr, std::string params_file, std::string params_position) override
    {
        cv::FileStorage fs(params_file, cv::FileStorage::READ);

        cv::FileNode tensor_shape_vector_fn = fs["post_processers"][params_position]["params"]["tensor_shape"];
        // if(tensor_shape_vector_fn.type() != cv::FileNode::SEQ) {}
        for (auto it = tensor_shape_vector_fn.begin(); it != tensor_shape_vector_fn.end(); it++) {          
            tensor_shape_.push_back((int)*it);
        }

        fs["post_processers"][params_position]["params"]["conf_thre"] >> conf_thre_;
        fs["post_processers"][params_position]["params"]["iou_thre"] >> iou_thre_;
        fs["post_processers"][params_position]["params"]["key_points_num"] >> key_points_num_;


        std::vector<int> remap_src_size;
        cv::FileNode remap_src_fn = fs["post_processers"][params_position]["params"]["remap_src"];
        // if(remap_src_fn.type() != cv::FileNode::SEQ) {}
        for (auto it = remap_src_fn.begin(); it != remap_src_fn.end(); it++) {          
            remap_src_size.push_back((int)*it);
        }

        std::vector<int> remap_dst_size;
        cv::FileNode remap_dst_fn = fs["post_processers"][params_position]["params"]["remap_dst"];
        // if(remap_dst_fn.type() != cv::FileNode::SEQ) {}
        for (auto it = remap_dst_fn.begin(); it != remap_dst_fn.end(); it++) {          
            remap_dst_size.push_back((int)*it);
        }

        remap_params_.scale = std::min((static_cast<double>(remap_src_size[0]) / static_cast<double>(remap_dst_size[0])), (static_cast<double>(remap_src_size[1]) / static_cast<double>(remap_dst_size[1])));
        remap_params_.ox = (-remap_params_.scale * remap_dst_size[0] + remap_src_size[0]) / 2.0;
        remap_params_.oy = (-remap_params_.scale * remap_dst_size[1] + remap_src_size[1]) / 2.0;

        p_tensor_ = reinterpret_cast<float*>(tensor_addr);
        // t_tensor_ = tensor_addr;
        // p_tensor_ = new float(tensor_shape_[1]*tensor_shape_[2]);
        

        // std::cout << metadata() << std::endl;

        return true;
    }

    std::string metadata() override
    {
        std::stringstream tensor_shape_stream;
        tensor_shape_stream << "[";
        for (size_t i = 0; i < tensor_shape_.size(); i++) {
            if (i != (tensor_shape_.size() - 1)) {
                tensor_shape_stream << tensor_shape_[i] << ", ";
            }
            else {
                tensor_shape_stream << tensor_shape_[i];
            }
        }
        tensor_shape_stream << "]";
        // std::copy(tensor_shape_.begin(), tensor_shape_.end(), std::ostreambuf_iterator<size_t>(tensor_shape_stream, ""));

        std::stringstream metadata_stream;
        metadata_stream
            << "INFO: (Fun) print_metadata() =>"                            << "\n" 
            << "================ post process metadata ================="   << "\n" 
            << "tensor shape : " << tensor_shape_stream.str()               << "\n"
            << "tensor_addr : " << (void*)p_tensor_                         << "\n" 
            << "conf_thre : " << conf_thre_                                 << "\n"
            << "iou_thre : " << iou_thre_                                   << "\n"
            << "key_points_num : " << key_points_num_                       << "\n"
            << "Remap Data : "                                              << "\n" 
            << "    scale : " << remap_params_.scale                        << "\n" 
            << "    ox : " << remap_params_.ox                              << "\n" 
            << "    oy : " << remap_params_.oy                              << "\n" 
            << "========================================================"   << "\n";

        return metadata_stream.str();
    }

    void process(DetectResultList & output) override 
    {
        /*
            [1 * 25200 * 51]
            x y w h conf x1 y1 x2 y2 x3 y3 x4 y4 x5 y5 [class conf]
        */     
        // memcpy(t_tensor_, p_tensor_, tensor_shape_[1]*tensor_shape_[2]*sizeof(float));

        DetectResultList pre_list;

        for (size_t i = 0; i < tensor_shape_[1]*tensor_shape_[2]; i+=tensor_shape_[2])
        {
            // bbox conf   
            // if (p_tensor_[i + 4] > 0.1) { std::cout << "fine: " << p_tensor_[i + 4] << std::endl; }         
            if (p_tensor_[i + 4] < conf_thre_) { continue; }

            

            // bbox conf * class conf
            double max_conf = 0.0;
            int max_id = 0;
            for (int j = 5 + key_points_num_ * 2; j < tensor_shape_[2]; j++)
            {
                if (p_tensor_[i + j] * p_tensor_[i + 4] >= max_conf)
                {
                    max_conf = p_tensor_[i + j] * p_tensor_[i + 4];
                    max_id = j - (5 + key_points_num_ * 2);
                }
            }
            if (max_conf <= conf_thre_) { continue; }

            // save result
            // DetectResult pre_result = {
            //     .bbox = cv::Rect2d(p_tensor_[i] - p_tensor_[i + 2] / 2.0, p_tensor_[i + 1] - p_tensor_[i + 3] / 2.0, p_tensor_[i + 2], p_tensor_[i + 3]),
            //     .points = {
            //             cv::Point2d(p_tensor_[i + 5], p_tensor_[i + 6]),
            //             cv::Point2d(p_tensor_[i + 7], p_tensor_[i + 8]),
            //             cv::Point2d(p_tensor_[i + 9], p_tensor_[i + 10]),
            //             cv::Point2d(p_tensor_[i + 11], p_tensor_[i + 12])
            //         },
            //     .id = max_id,
            //     .conf = max_conf
            // };
            DetectResult pre_result = {
                .bbox = cv::Rect2d(
                        ((p_tensor_[i] - p_tensor_[i + 2] / 2.0) - remap_params_.ox) / remap_params_.scale, 
                        ((p_tensor_[i + 1] - p_tensor_[i + 3] / 2.0) - remap_params_.oy) / remap_params_.scale, 
                        p_tensor_[i + 2] / remap_params_.scale, 
                        p_tensor_[i + 3] / remap_params_.scale
                    ),
                .points = {
                        cv::Point2d(
                                (p_tensor_[i + 5] - remap_params_.ox) / remap_params_.scale, 
                                (p_tensor_[i + 6] - remap_params_.oy) / remap_params_.scale
                            ),
                        cv::Point2d(
                                (p_tensor_[i + 7] - remap_params_.ox) / remap_params_.scale, 
                                (p_tensor_[i + 8] - remap_params_.oy) / remap_params_.scale
                            ),
                        cv::Point2d(
                                (p_tensor_[i + 9] - remap_params_.ox) / remap_params_.scale, 
                                (p_tensor_[i + 10] - remap_params_.oy) / remap_params_.scale
                            ),
                        cv::Point2d(
                                (p_tensor_[i + 11] - remap_params_.ox) / remap_params_.scale, 
                                (p_tensor_[i + 12] - remap_params_.oy) / remap_params_.scale
                            )
                    },
                .id = max_id,
                .conf = max_conf
            };
            pre_list.push_back(pre_result);
        }

        non_max_suppression(pre_list, output, iou_thre_);


        // std::cout << "=============================" << std::endl;
        // std::cout << pre_list.size() << std::endl;
        // std::cout << output.size() << std::endl;

        // for (auto result : output) {
        //     std::cout << "id: " << result.id << " conf: " << result.conf
        //     << " bbox:[" << result.bbox.x << ", " << result.bbox.y << "][" << result.bbox.width << ", " << result.bbox.height << "], "
        //     << "points:[" << result.points[0].x << ", " << result.points[0].y << "]["
        //     << result.points[1].x << ", " << result.points[1].y << "]["
        //     << result.points[2].x << ", " << result.points[2].y << "]["
        //     << result.points[3].x << ", " << result.points[3].y << "][" 
        //     << result.points[4].x << ", " << result.points[4].y << "]["<< std::endl;
        // }

    }


    void process(SegmentResultList & output) override {}



    RmV5PostProcesser() {}
    ~RmV5PostProcesser() {}

private:
    bool non_max_suppression(DetectResultList &pre_list, DetectResultList &post_list, const double iou_threshold = 0.5)
    {
        post_list.clear();
        std::sort(pre_list.begin(), pre_list.end(), [](DetectResult &a, DetectResult &b) { return bool(a.conf > b.conf); });
        while (pre_list.size() > 0)
        {
            post_list.push_back(pre_list.at(0));
            int index = 1;
            while (index < pre_list.size())
            {
                if (pre_list[0].id != pre_list.at(index).id)
                {
                    index++;
                    continue;
                }
                float iou_value = getIou(pre_list.at(0).bbox, pre_list.at(index).bbox);
                if (iou_value > iou_threshold)
                    pre_list.erase(pre_list.begin() + index);
                else
                    index++;
            }
            pre_list.erase(pre_list.begin());
        }
        return true;
    }

    double getIou(cv::Rect2d &a, cv::Rect2d &b)
    {
        cv::Rect2d same_rect = a & b;
        return same_rect.area() / (a.area() + b.area() - same_rect.area());
    }


private:
    float* p_tensor_;
    void* t_tensor_;

    std::vector<size_t> tensor_shape_;

    double conf_thre_ = 0.5;
    double iou_thre_ = 0.4;
    int key_points_num_ = 5;

    struct RemapParams
    {
        double scale;
        double ox;
        double oy;
    } remap_params_;

};





#endif