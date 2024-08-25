#ifndef NCNNMODEL_HPP
#define NCNNMODEL_HPP

#include <cmath>
#include <algorithm>
#include "net.h"
#include "benchmark.h"
#include "cpu.h"
#include <opencv2/opencv.hpp>
#include "readConfig.hpp"

class TargetBox {
private:
    [[nodiscard]] float GetWidth() const { return static_cast<float>(x2 - x1); };
    [[nodiscard]] float GetHeight() const { return static_cast<float>(y2 - y1); };

public:
    int x1;
    int y1;
    int x2;
    int y2;

    int category;
    float score;

    float area() { return GetWidth() * GetHeight(); };
};

class ObjectDetector {
public:
    ObjectDetector(const char* model_param_path, const char* model_bin_path, int input_width, int input_height)
        : input_width(input_width), input_height(input_height), net(std::make_unique<ncnn::Net>()) {
        if(project_config.ARM) {
            ncnn::set_cpu_powersave(2);
            net->opt.num_threads = 2;
            net->opt.use_packing_layout = true;
            net->opt.use_bf16_storage = true;
        } else {
            net->opt.use_packing_layout = true;
            net->opt.num_threads = 1;
        }
        net->load_param(model_param_path);
        net->load_model(model_bin_path);
    }

    std::pair<float, float> detectLoop(cv::Mat& frame) {
        ncnn::Mat input = ncnn::Mat::from_pixels_resize(frame.data, ncnn::Mat::PIXEL_BGR, frame.cols, frame.rows, input_width, input_height);

        std::pair<float, float> max_center_points;

        int img_width = frame.cols;
        int img_height = frame.rows;

        const float mean_vals[3] = {0.f, 0.f, 0.f};
        const float norm_vals[3] = {1/255.f, 1/255.f, 1/255.f};
        input.substract_mean_normalize(mean_vals, norm_vals); 

        ncnn::Extractor ex = net->create_extractor();

        ex.set_num_threads(1);
        ex.input("input.1", input);

        input.release();

        ncnn::Mat output; 
        ex.extract("758", output); 

        std::vector<TargetBox> target_boxes;

        for (int h = 0; h < output.h; h++) {
            for (int w = 0; w < output.w; w++) {
                int obj_score_index_h = (0 * output.h * output.w) + (h * output.w) + w;
                float obj_score = output[obj_score_index_h];

                int category;
                float max_score = 0.0f;

                for (size_t i = 0; i < class_num; i++) {
                    int obj_score_index_i = static_cast<int>(((5 + i) * output.h * output.w) + (h * output.w) + w);
                    float cls_score = output[obj_score_index_i];
                    if (cls_score > max_score) {
                        max_score = cls_score;
                        category = static_cast<int>(i);
                    }
                }
                auto score = static_cast<float>(pow(max_score, 0.4) * pow(obj_score, 0.6));

                if(score > thresh) {
                    int x_offset_index = (1 * output.h * output.w) + (h * output.w) + w;
                    int y_offset_index = (2 * output.h * output.w) + (h * output.w) + w;
                    int box_width_index = (3 * output.h * output.w) + (h * output.w) + w;
                    int box_height_index = (4 * output.h * output.w) + (h * output.w) + w;    

                    float x_offset = Tanh(output[x_offset_index]);
                    float y_offset = Tanh(output[y_offset_index]);
                    float box_width = Sigmoid(output[box_width_index]);
                    float box_height = Sigmoid(output[box_height_index]);

                    float cx = (static_cast<float>(w) + x_offset) / static_cast<float>(output.w);
                    float cy = (static_cast<float>(h) + y_offset) / static_cast<float>(output.h);

                    int x1 = (int)((cx - box_width * 0.5) * img_width);
                    int y1 = (int)((cy - box_height * 0.5) * img_height);
                    int x2 = (int)((cx + box_width * 0.5) * img_width);
                    int y2 = (int)((cy + box_height * 0.5) * img_height);

                    target_boxes.push_back(TargetBox{x1, y1, x2, y2, category, score});
                }
            }
        }

        std::vector<TargetBox> nms_boxes;
        nmsHandle(target_boxes, nms_boxes);

        for(auto box : nms_boxes) {
            std::string object_name = class_names[box.category];
            int percentage = static_cast<int>(box.score * 100);
            std::string label = object_name + ": " + std::to_string(percentage) + "%";

            cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.7, 1, nullptr);

            int label_ymin = std::max(box.y1, labelSize.height + 10);

            cv::rectangle(frame, cv::Point(box.x1, box.y1), cv::Point(box.x2, box.y2), cv::Scalar(10, 255, 0), 2);

            cv::rectangle(frame, cv::Point(box.x1, label_ymin - labelSize.height - 10), 
                        cv::Point(box.x1 + labelSize.width, label_ymin + labelSize.height - 15), 
                        cv::Scalar(255, 255, 255), cv::FILLED);

            cv::putText(frame, label, cv::Point(box.x1, label_ymin - 7), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 1);

        }

        if(!nms_boxes.empty()) {
            TargetBox& highest_score_box = nms_boxes[0];
            max_center_points.first = 0.5f * static_cast<float>(highest_score_box.x1 + highest_score_box.x2) / static_cast<float>(img_width);
            max_center_points.second = 0.5f * static_cast<float>(highest_score_box.y1 + highest_score_box.y2) / static_cast<float>(img_height);
        }

        if(project_config.ShowCameraFrames) {
            cv::imshow("Result", frame);
        }
        
        return max_center_points;
    }

private:
    int input_width;
    int input_height;
    std::unique_ptr<ncnn::Net> net;
    std::vector<std::string> class_names = {"bottle"};

    int class_num = static_cast<int>(class_names.size());
    float thresh = 0.7;
    Config project_config = readYAML();

    static float Sigmoid(float x) {
        return 1.0f / (1.0f + exp(-x));
    }

    static float Tanh(float x) {
        return 2.0f / (1.0f + exp(-2 * x)) - 1;
    }

    static float IntersectionArea(const TargetBox &a, const TargetBox &b) {
        if (a.x1 > b.x2 || a.x2 < b.x1 || a.y1 > b.y2 || a.y2 < b.y1) {
            return 0.f;
        }

        auto inter_width = static_cast<float>(std::min(a.x2, b.x2) - std::max(a.x1, b.x1));
        auto inter_height = static_cast<float>(std::min(a.y2, b.y2) - std::max(a.y1, b.y1));

        return inter_width * inter_height;
    }

    static bool scoreSort(TargetBox a, TargetBox b) { 
        return (a.score > b.score); 
    }

    static int nmsHandle(std::vector<TargetBox> &src_boxes, std::vector<TargetBox> &dst_boxes) {
        std::vector<int> picked;
        
        sort(src_boxes.begin(), src_boxes.end(), scoreSort);

        for(int i = 0; i < src_boxes.size(); i++) {
            int keep = 1;

            for(int j : picked) {
                float inter_area = IntersectionArea(src_boxes[i], src_boxes[j]);
                float union_area = src_boxes[i].area() + src_boxes[j].area() - inter_area;
                float IoU = inter_area / union_area;

                if(IoU > 0.45 && src_boxes[i].category == src_boxes[j].category) {
                    keep = 0;
                    break;
                }
            }

            if (keep) {
                picked.push_back(i);
            }
        }
        
        for(int i : picked) {
            dst_boxes.push_back(src_boxes[i]);
        }
        src_boxes.clear();

        return 0;
    }
};

#endif //NCNNMODEL_HPP
