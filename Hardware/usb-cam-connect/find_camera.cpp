#include <iostream>
#include <string>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <array>
#include <vector>
#include <map>
#include <filesystem> 
#include <regex>


namespace fs = std::filesystem;

// 接口函数：查找符合要求的USB相机，并返回索引
int initCamera(){
    std::map<std::string, std::string> real_cam_attrs = {
        {"name", "DECXIN Camera : DECXIN Camera "},
        {"index", "0"}
    };
    int real_cam_index = findCameraIndexByAttributes(real_cam_attrs);

    // 3. 依次查询
    int camera_to_use = -1;
    if (real_cam_index != -1) {
        std::cout << "检测到真实硬件，使用USB摄像头。" << std::endl;
        camera_to_use = real_cam_index;
    } else {
        std::cout << "错误：未找到任何配置的摄像头！" << std::endl;
    }

    return camera_to_use;
}

// 辅助函数：执行一个shell命令并返回其标准输出
std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

// 主函数：通过属性查找摄像头索引
int findCameraIndexByAttributes(const std::map<std::string, std::string>& attrs_to_match) {
    std::regex video_pattern("video(\\d+)");
    
    for (const auto& entry : fs::directory_iterator("/dev")) {
        const std::string path = entry.path().string();
        if (path.find("/dev/video") != std::string::npos) {
            std::string command = "udevadm info -a -n " + path;
            std::string all_attributes;
            try {
                all_attributes = exec(command.c_str());
            } catch(...) {
                continue; // 如果命令失败，跳过这个设备
            }

            bool found_all = true;
            for (const auto& pair : attrs_to_match) {
                // 构建正则表达式来匹配 ATTRS{key}=="value"
                std::string pattern_str = "ATTRS?\\{" + pair.first + "\\}==\"" + pair.second + "\"";
                std::regex attr_pattern(pattern_str);
                if (!std::regex_search(all_attributes, attr_pattern)) {
                    found_all = false;
                    break;
                }
            }

            if (found_all) {
                std::smatch match;
                if (std::regex_search(path, match, video_pattern)) {
                    int index = std::stoi(match[1].str());
                    //std::cout << "找到匹配设备 at " << path << " (索引: " << index << ")" << std::endl;
                    return index;
                }
            }
        }
    }
    //std::cerr << "错误: 未找到匹配指定属性的摄像头。" << std::endl;
    return -1; // -1 表示未找到
}