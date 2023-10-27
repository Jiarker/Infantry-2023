#ifndef CV_PRE_PROCESSING_
#define CV_PRE_PROCESSING_

#include <opencv4/opencv2/opencv.hpp>

#include <iostream>
#include <vector>
#include <memory>
#include <typeinfo>

#include "Module.hpp"

class CvStandardAffinePreProcesser : public PreProcesser
{
public:
    CvStandardAffinePreProcesser() {}
    ~CvStandardAffinePreProcesser() {}
    bool init(
            cv::Size camera_size, cv::Size net_input_size, 
            double normalize_scale = 255.0, int normalize_bias = 0,
            bool bind_uesr_tensor_addr = false, void* tensor_addr = nullptr, int tensor_type = CV_32FC3
        ) override 
    {
        if (camera_size.empty() || net_input_size.empty()) {
            error_messages_ = "PreProcesser ERROR : Init with empty size";
            return false;
        }

        if (normalize_scale != 255.0  ||  normalize_bias != 0) {
            error_messages_ = "PreProcesser ERROR : Init standard prep with unstandard normalize params";
            return false;
        }
        
        
        affine_params_.scale = std::min((static_cast<double>(net_input_size.width) / static_cast<double>(camera_size.width)), (static_cast<double>(net_input_size.height) / static_cast<double>(camera_size.height)));
        affine_params_.ox = (-affine_params_.scale * camera_size.width + net_input_size.width)  / 2;
        affine_params_.oy = (-affine_params_.scale * camera_size.height + net_input_size.height) / 2;
        affine_params_.size = net_input_size;

        affine_matrixa_ = (
                cv::Mat_<double>(2, 3) << 
                    affine_params_.scale,   0,                      affine_params_.ox, 
                    0,                      affine_params_.scale,   affine_params_.oy
            );

        // cv::Point2f src[3] = {  
        //         cv::Point2d(camera_size.width, 0), 
        //         cv::Point2d(0, camera_size.height), 
        //         cv::Point2d(camera_size.width, camera_size.height)
        //     };

        // cv::Point2f dst[3] = {  
        //         cv::Point2d(camera_size.width * affine_params_.scale, 0), 
        //         cv::Point2d(0, camera_size.height * affine_params_.scale), 
        //         cv::Point2d(camera_size.width * affine_params_.scale, camera_size.height * affine_params_.scale)
        //     };

        // affine_matrixa_ = cv::getAffineTransform(src, dst);

        tensor_type_ = tensor_type;

        preprocessing_img_ = cv::Mat(net_input_size.height, net_input_size.width, CV_8UC3);

        if (bind_uesr_tensor_addr == true) {
            if (tensor_addr == nullptr) {
                error_messages_ = "PreProcesser ERROR : Init bind with nullptr";
                return false;
            }

            output_tensor_img_ = cv::Mat(
                    net_input_size.height,
                    net_input_size.width, 
                    tensor_type_, 
                    tensor_addr
                );
        }
        else {
            // output_tensor_img_ = cv::Mat(
            //     net_input_size.height, 
            //     net_input_size.width, 
            //     tensor_type_
            // );

            cv::dnn::blobFromImage(preprocessing_img_, output_tensor_img_, 1/255.0, net_input_size, cv::Scalar(0, 0, 0), true, false);
        }

        return true;

    }

    std::string metadata() override {
        std::stringstream metadata;
        metadata
            << "INFO: (Fun) print_metadata() =>"                            << "\n" 
            << "================= pre process metadata ================="   << "\n" 
            << "Affine Data : "                                             << "\n" 
            << "    scale : " << affine_params_.scale                       << "\n" 
            << "    ox : " << affine_params_.ox                             << "\n" 
            << "    oy : " << affine_params_.oy                             << "\n" 
            << "    size : "                                                << "\n" 
            << "        w : " << affine_params_.size.width                  << "\n" 
            << "        h : " << affine_params_.size.height                 << "\n" 
            << "    value : " << affine_params_.value                       << "\n"
            << "tensor_addr : " << (void*)output_tensor_img_.data           << "\n" 
            << "tensor_type : " << tensor_type_                             << "\n"
            << "========================================================"   << "\n";

        return metadata.str();
    }

    bool init_with_log(
            cv::Size camera_size, cv::Size net_input_size, 
            double normalize_scale = 255.0, int normalize_bias = 0,
            bool bind_uesr_tensor_addr = false, void* tensor_addr = nullptr, int tensor_type = CV_32FC3
        ) override 
    {
        bool flag = init(
                camera_size, net_input_size, normalize_scale, normalize_bias,
                bind_uesr_tensor_addr, tensor_addr, tensor_type
            );
        if (flag == false) {
            std::cout << "Init ERROR Log : " << std::endl;
            std::cout << error_messages() << std::endl;
        }
        else {
            std::cout << "Init SUCCESS Log : " << std::endl;
            std::cout << metadata() << std::endl;
        }

        return flag;
    }

    bool init_with_yaml(std::string params_file, std::string params_position) override
    {
        cv::FileStorage fs(params_file, cv::FileStorage::READ);

        std::vector<size_t> camera_size, tensor_shape;

        cv::FileNode camera_size_vector_fn = fs["pre_processers"][params_position]["params"]["camera_size"];
        
        // if(camera_size_vector_fn.type() != cv::FileNode::SEQ) {
        //     std::cout << "error type : " << camera_size_vector_fn.type() << std::endl;
        // }

        for (auto it = camera_size_vector_fn.begin(); it != camera_size_vector_fn.end(); it++) {          
            camera_size.push_back((int)*it);
        }

        cv::FileNode tensor_shape_vector_fn = fs["pre_processers"][params_position]["params"]["tensor_shape"];
        // if(tensor_shape_vector_fn.type() != cv::FileNode::SEQ) {}
        for (auto it = tensor_shape_vector_fn.begin(); it != tensor_shape_vector_fn.end(); it++) {          
            tensor_shape.push_back((int)*it);
        }

        // fs["pre_processers"][params_position]["params"]["camera_size"] >> camera_size;
        // fs["pre_processers"][params_position]["params"]["tensor_shape"] >> tensor_shape;

        // return init_with_log(
        //         {camera_size[0], camera_size[1]},
        //         {tensor_shape[3], tensor_shape[2]}
        //     );

        return init(
                {camera_size[0], camera_size[1]},
                {tensor_shape[3], tensor_shape[2]}
            );
    }

    std::string error_messages() override {
        return error_messages_;
    }

    cv::Mat preprocessing_img() override {
        return preprocessing_img_;
    }

    void process(cv::Mat & image) override 
    {

        cv::warpAffine(image, preprocessing_img_, 
                affine_matrixa_, 
                affine_params_.size, cv::INTER_LINEAR,
                cv::BORDER_CONSTANT, affine_params_.value
            );

        cv::dnn::blobFromImage(preprocessing_img_, output_tensor_img_, 1/255.0, affine_params_.size, cv::Scalar(0, 0, 0), true, false);


    }

    std::vector<void*> outputs() override
    {
        std::vector<void*> outputs_addr;
        outputs_addr.push_back(reinterpret_cast<void*>(output_tensor_img_.data));
        return outputs_addr;
    }

private:

    inline static void hwc_to_chw(cv::InputArray src, cv::OutputArray dst) {
        std::vector<cv::Mat> channels;
        cv::split(src, channels);

        // Stretch one-channel images to vector
        for (auto &img : channels) {
            img = img.reshape(1, 1);
        }

        // Concatenate three vectors to one
        cv::hconcat( channels, dst );
    }

private:
    struct AffineParams
    {
        double scale;
        double ox;
        double oy;
        cv::Size size;
        cv::Scalar value = {114, 114, 114};
    } affine_params_;

    cv::Mat affine_matrixa_;

    int tensor_type_;

    cv::Mat preprocessing_img_;

    cv::Mat output_tensor_img_;

    std::string error_messages_;

    
};


























// template<typename T>
// class CvPreProcesser
// {

// public:
    
//     inline void init(
//             cv::Size camera_size, cv::Size net_input_size, bool keep_ratio,
//             double normalize_scale, int normalize_bias,
//             bool bind_uesr_tensor_addr, T* tensor_addr
//         )
//     {
        
//         // 1. 
//         if (keep_ratio) {
//             double hw_scale = static_cast<double>(camera_size.height) / static_cast<double>(camera_size.width);
//             if (hw_scale > 1) {
//                 resize_size_ = {(net_input_size.width / hw_scale), net_input_size.height};
//                 need_padding_ = true;
//                 padding_.left = static_cast<int>((net_input_size.width - static_cast<int>(net_input_size.width / hw_scale)) * 0.5);
//                 padding_.right = static_cast<int>(net_input_size.width - static_cast<int>(net_input_size.width / hw_scale)) - padding_.left;
//                 padding_.top = 0;
//                 padding_.bottom = 0;
//             }
//             else if (hw_scale < 1) {
//                 resize_size_ = {net_input_size.width, (net_input_size.height * hw_scale)};
//                 need_padding_ = true;
//                 padding_.left = 0;
//                 padding_.right = 0;
//                 padding_.top = static_cast<int>((net_input_size.height - static_cast<int>(net_input_size.height * hw_scale)) * 0.5);
//                 padding_.bottom = static_cast<int>(net_input_size.height - static_cast<int>(net_input_size.height * hw_scale)) - padding_.top;
//             }
//             else {
//                 resize_size_ = net_input_size;
//                 need_padding_ = false;
//                 padding_.left = 0;
//                 padding_.right = 0;
//                 padding_.top = 0;
//                 padding_.bottom = 0;
//             }
//         }
//         else {
//             resize_size_ = net_input_size;
//             need_padding_ = false;
//             padding_.left =auto p = prep->process(frame); 0;
//             padding_.right = 0;
//             padding_.top = 0;
//             padding_.bottom = 0;
//         }

//         normalize_scale_ = normalize_scale;
//         normalize_bias_ = normalize_bias;
//         if (bind_uesr_tensor_addr == true) {
//             tensor_addr_ = tensor_addr;
//         }
//         else {
//             tensor_addr_ = new T(3 * net_input_size.height * net_input_size.width);
//         }

//         if (normalize_bias_ == 0  &&  normalize_scale_ == 255  
//             &&  typeid(tensor_addr_) == typeid(float*)) {
//             // use_standard_normalize_ = true;
//             use_standard_normalize_ = false;
//         }
//         else {
//             use_standard_normalize_ = false;
//         }

//         affine_scale_ = std::min((static_cast<double>(net_input_size.width) / static_cast<double>(camera_size.width)), (static_cast<double>(net_input_size.height) / static_cast<double>(camera_size.height)));
//         affine_ox_ = (-affine_scale_ * camera_size.width + net_input_size.width)  / 2;
//         affine_oy_ = (-affine_scale_ * camera_size.height + net_input_size.height) / 2;
//         affine_size_ = net_input_size;

//     }

//     inline void print_metadata() 
//     {
//         std::cout << "INFO: (Fun) print_metadata() =>" << std::endl;
//         std::cout << "================= pre process metadata =================" << std::endl;
//         std::cout << "resize_size : " << resize_size_ << std::endl;
//         std::cout << "resize_flag : " << resize_flag_ << std::endl;
//         std::cout << "need_padding : " << need_padding_ << std::endl;
//         std::cout << "Padding Data : " << need_padding_ << std::endl;
//         std::cout << "     top : " << padding_.top << std::endl;
//         std::cout << "     bottom : " << padding_.bottom << std::endl;
//         std::cout << "     left : " << padding_.left << std::endl;
//         std::cout << "     right : " << padding_.right << std::endl;
//         std::cout << "     type : " << padding_.type << std::endl;
//         std::cout << "     value : " << padding_.value << std::endl;
//         std::cout << "normalize_scale : " << normalize_scale_ << std::endl;
//         std::cout << "normalize_bias : " << normalize_bias_ << std::endl;
//         std::cout << "bind_tensor_addr : " << bind_tensor_addr_ << std::endl;
//         std::cout << "tensor_addr : " << tensor_addr_ << std::endl;
//         std::cout << "tensor_type : " << typeid(tensor_addr_).name() << std::endl;
//         std::cout << "use_standard_normalize : " << use_standard_normalize_ << std::endl;
//         std::cout << "========================================================" << std::endl;
//     }

//     void show_preprocessing_img()
//     {

//     }

//     T* process(cv::Mat & image)
//     {
//         // 1.
//         cv::Mat affine_matrixa = (
//                 cv::Mat_<double>(2, 3) << 
//                     affine_scale_,  0,              affine_ox_, 
//                     0,              affine_scale_,  affine_oy_
//             );

//         cv::Mat img_resized_padding;
//         cv::warpAffine(image, img_resized_padding, 
//                 affine_matrixa, 
//                 affine_size_, cv::INTER_LINEAR,
//                 cv::BORDER_CONSTANT, padding_.value
//             );


//         cv::Mat img_resized, img_resized_padding;
//         cv::resize(image, img_resized, resize_size_, resize_flag_);
//         if (need_padding_) {
//             cv::copyMakeBorder(img_resized, img_resized_padding, 
//                     padding_.top, padding_.bottom, padding_.left, padding_.right, 
//                     padding_.type, padding_.value
//                 );
//         }
//         else {
//             img_resized_padding = img_resized;
//         }

//         // 2. normalize and set input tensor       

//         // Standard Bias & Scale Transform
//         if (use_standard_normalize_) {
            
//             cv::Mat img_resized_padding_rgb;

//             cv::cvtColor(img_resized_padding, img_resized_padding_rgb, cv::COLOR_BGR2RGB);

//             auto tensor_mat = cv::Mat(
//                     img_resized_padding_rgb.rows, 
//                     img_resized_padding_rgb.cols, 
//                     CV_32FC3, 
//                     tensor_addr_
//                 );

//             img_resized_padding_rgb.convertTo(tensor_mat, CV_32FC3, 1/255.0);

//             return tensor_addr_;
//         }
        
//         // Other Bias & Scale Transform

        
        
//         for (size_t c = 0; c < 3; c++)  // bgr
//         {
//             for (int i = 0; i < img_resized_padding.rows; i++)  // 行
//             {
//                 for (int j = 0; j < img_resized_padding.cols; j++)  // 列
//                 {
//                     // Mat里的ptr函数访问任意一行像素的首地址,2-c:表示rgb
//                     tensor_addr_[c * img_resized_padding.rows * img_resized_padding.cols + i * img_resized_padding.cols + j] = 
//                             (img_resized_padding.ptr<uchar>(i)[j * 3 + 2 - c] - normalize_bias_) / normalize_scale_;
//                 }
//             }
//         }

//         return tensor_addr_;
//     }
    
// public:
//     CvPreProcesser(
//             cv::Size camera_size, cv::Size net_input_size, bool keep_ratio,
//             double normalize_scale, int normalize_bias,
//             bool bind_tensor_addr, T* tensor_addr
//         )
//     {
//         init(camera_size, net_input_size, keep_ratio,normalize_scale, normalize_bias, bind_tensor_addr, tensor_addr);
//     }

//     CvPreProcesser() {}

//     ~CvPreProcesser() {
//         if (bind_tensor_addr_) {
//             delete(tensor_addr_);
//         }
//     }

// private:

//     cv::Size resize_size_;
//     cv::InterpolationFlags resize_flag_ = cv::INTER_AREA;

//     double affine_scale_;
//     double affine_ox_;
//     double affine_oy_;
//     cv::Size affine_size_;

//     bool need_padding_ = true;
//     struct Padding
//     {
//         int top; int bottom; int left; int right;
//         cv::BorderTypes type = cv::BORDER_CONSTANT; cv::Scalar value = {114, 114, 114};
//     } padding_;

//     double normalize_scale_;
//     int normalize_bias_;

//     bool bind_tensor_addr_;
//     T* tensor_addr_;







// template<typename T>
// class CvPreProcesser
// {

// public:
    
//     inline void init(
//             cv::Size camera_size, cv::Size net_input_size, bool keep_ratio,
//             double normalize_scale, int normalize_bias,
//             bool bind_uesr_tensor_addr, T* tensor_addr
//         )
//     {
        
//         // 1. 
//         if (keep_ratio) {
//             double hw_scale = static_cast<double>(camera_size.height) / static_cast<double>(camera_size.width);
//             if (hw_scale > 1) {
//                 resize_size_ = {(net_input_size.width / hw_scale), net_input_size.height};
//                 need_padding_ = true;
//                 padding_.left = static_cast<int>((net_input_size.width - static_cast<int>(net_input_size.width / hw_scale)) * 0.5);
//                 padding_.right = static_cast<int>(net_input_size.width - static_cast<int>(net_input_size.width / hw_scale)) - padding_.left;
//                 padding_.top = 0;
//                 padding_.bottom = 0;
//             }
//             else if (hw_scale < 1) {
//                 resize_size_ = {net_input_size.width, (net_input_size.height * hw_scale)};
//                 need_padding_ = true;
//                 padding_.left = 0;
//                 padding_.right = 0;
//                 padding_.top = static_cast<int>((net_input_size.height - static_cast<int>(net_input_size.height * hw_scale)) * 0.5);
//                 padding_.bottom = static_cast<int>(net_input_size.height - static_cast<int>(net_input_size.height * hw_scale)) - padding_.top;
//             }
//             else {
//                 resize_size_ = net_input_size;
//                 need_padding_ = false;
//                 padding_.left = 0;
//                 padding_.right = 0;
//                 padding_.top = 0;
//                 padding_.bottom = 0;
//             }
//         }
//         else {
//             resize_size_ = net_input_size;
//             need_padding_ = false;
//             padding_.left = 0;
//             padding_.right = 0;
//             padding_.top = 0;
//             padding_.bottom = 0;
//         }

//         normalize_scale_ = normalize_scale;
//         normalize_bias_ = normalize_bias;
//         if (bind_uesr_tensor_addr == true) {
//             tensor_addr_ = tensor_addr;
//         }
//         else {
//             tensor_addr_ = new T(3 * net_input_size.height * net_input_size.width);
//         }

//         if (normalize_bias_ == 0  &&  normalize_scale_ == 255  
//             &&  typeid(tensor_addr_) == typeid(float*)) {
//             // use_standard_normalize_ = true;
//             use_standard_normalize_ = false;
//         }
//         else {
//             use_standard_normalize_ = false;
//         }

//         affine_scale_ = std::min((static_cast<double>(net_input_size.width) / static_cast<double>(camera_size.width)), (static_cast<double>(net_input_size.height) / static_cast<double>(camera_size.height)));
//         affine_ox_ = (-affine_scale_ * camera_size.width + net_input_size.width)  / 2;
//         affine_oy_ = (-affine_scale_ * camera_size.height + net_input_size.height) / 2;
//         affine_size_ = net_input_size;

//     }

//     inline void print_metadata() 
//     {
//         std::cout << "INFO: (Fun) print_metadata() =>" << std::endl;
//         std::cout << "================= pre process metadata =================" << std::endl;
//         std::cout << "resize_size : " << resize_size_ << std::endl;
//         std::cout << "resize_flag : " << resize_flag_ << std::endl;
//         std::cout << "need_padding : " << need_padding_ << std::endl;
//         std::cout << "Padding Data : " << need_padding_ << std::endl;
//         std::cout << "     top : " << padding_.top << std::endl;
//         std::cout << "     bottom : " << padding_.bottom << std::endl;
//         std::cout << "     left : " << padding_.left << std::endl;
//         std::cout << "     right : " << padding_.right << std::endl;
//         std::cout << "     type : " << padding_.type << std::endl;
//         std::cout << "     value : " << padding_.value << std::endl;
//         std::cout << "normalize_scale : " << normalize_scale_ << std::endl;
//         std::cout << "normalize_bias : " << normalize_bias_ << std::endl;
//         std::cout << "bind_tensor_addr : " << bind_tensor_addr_ << std::endl;
//         std::cout << "tensor_addr : " << tensor_addr_ << std::endl;
//         std::cout << "tensor_type : " << typeid(tensor_addr_).name() << std::endl;
//         std::cout << "use_standard_normalize : " << use_standard_normalize_ << std::endl;
//         std::cout << "========================================================" << std::endl;
//     }

//     void show_preprocessing_img()
//     {

//     }

//     T* process(cv::Mat & image)
//     {
//         // 1.
//         cv::Mat affine_matrixa = (
//                 cv::Mat_<double>(2, 3) << 
//                     affine_scale_,  0,              affine_ox_, 
//                     0,              affine_scale_,  affine_oy_
//             );

//         cv::Mat img_resized_padding;
//         cv::warpAffine(image, img_resized_padding, 
//                 affine_matrixa, 
//                 affine_size_, cv::INTER_LINEAR,
//                 cv::BORDER_CONSTANT, padding_.value
//             );


//         cv::Mat img_resized, img_resized_padding;
//         cv::resize(image, img_resized, resize_size_, resize_flag_);
//         if (need_padding_) {
//             cv::copyMakeBorder(img_resized, img_resized_padding, 
//                     padding_.top, padding_.bottom, padding_.left, padding_.right, 
//                     padding_.type, padding_.value
//                 );
//         }
//         else {
//             img_resized_padding = img_resized;
//         }

//         // 2. normalize and set input tensor       

//         // Standard Bias & Scale Transform
//         if (use_standard_normalize_) {
            
//             cv::Mat img_resized_padding_rgb;

//             cv::cvtColor(img_resized_padding, img_resized_padding_rgb, cv::COLOR_BGR2RGB);

//             auto tensor_mat = cv::Mat(
//                     img_resized_padding_rgb.rows, 
//                     img_resized_padding_rgb.cols, 
//                     CV_32FC3, 
//                     tensor_addr_
//                 );

//             img_resized_padding_rgb.convertTo(tensor_mat, CV_32FC3, 1/255.0);

//             return tensor_addr_;
//         }
        
//         // Other Bias & Scale Transform

        
        
//         for (size_t c = 0; c < 3; c++)  // bgr
//         {
//             for (int i = 0; i < img_resized_padding.rows; i++)  // 行
//             {
//                 for (int j = 0; j < img_resized_padding.cols; j++)  // 列
//                 {
//                     // Mat里的ptr函数访问任意一行像素的首地址,2-c:表示rgb
//                     tensor_addr_[c * img_resized_padding.rows * img_resized_padding.cols + i * img_resized_padding.cols + j] = 
//                             (img_resized_padding.ptr<uchar>(i)[j * 3 + 2 - c] - normalize_bias_) / normalize_scale_;
//                 }
//             }
//         }

//         return tensor_addr_;
//     }
    
// public:
//     CvPreProcesser(
//             cv::Size camera_size, cv::Size net_input_size, bool keep_ratio,
//             double normalize_scale, int normalize_bias,
//             bool bind_tensor_addr, T* tensor_addr
//         )
//     {
//         init(camera_size, net_input_size, keep_ratio,normalize_scale, normalize_bias, bind_tensor_addr, tensor_addr);
//     }

//     CvPreProcesser() {}

//     ~CvPreProcesser() {
//         if (bind_tensor_addr_) {
//             delete(tensor_addr_);
//         }
//     }

// private:

//     cv::Size resize_size_;
//     cv::InterpolationFlags resize_flag_ = cv::INTER_AREA;

//     double affine_scale_;
//     double affine_ox_;
//     double affine_oy_;
//     cv::Size affine_size_;

//     bool need_padding_ = true;
//     struct Padding
//     {
//         int top; int bottom; int left; int right;
//         cv::BorderTypes type = cv::BORDER_CONSTANT; cv::Scalar value = {114, 114, 114};
//     } padding_;

//     double normalize_scale_;
//     int normalize_bias_;

//     bool bind_tensor_addr_;
//     T* tensor_addr_;

//     bool use_standard_normalize_;

    
// };


#endif