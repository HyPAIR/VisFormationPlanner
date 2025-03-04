/**
 * file yaml.cpp
 * author Weijian Zhang (wxz163@student.bham.ac.uk)
 * brief yaml
 * data 2023-11-22
 * 
 * @copyright Copyroght(c) 2023
*/
#include "vis_formation_planner/yaml_all.h"

namespace vis_formation_planner {

bool isFileEmpty(const std::string& filename) {
    std::ifstream file(filename);
    return file.peek() == std::ifstream::traits_type::eof();
}

void clearYAMLFile(const std::string& file_path) {
    try {
        // 以截断模式打开文件，这会清空文件内容
        std::ofstream file_out(file_path, std::ofstream::out | std::ofstream::trunc);
        file_out.close(); // 关闭文件
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}

void writePriorityToYAML(const std::vector<std::vector<int>>& priority_set_all, const std::string& filename) {
    if (!isFileEmpty(filename)) {
        std::cout << "File '" << filename << "' is not empty. Not writing again." << std::endl;
        return;
    }
    YAML::Node yamlNode;

    for (const auto& row : priority_set_all) {
        YAML::Node rowNode;

        for (const int& value : row) {
            rowNode.push_back(value);
        }

        yamlNode.push_back(rowNode);
    }

    // 将 YAML 节点写入文件
    std::ofstream file(filename);
    file << yamlNode;
    file.close();
    file.close();;

    std::cout << "Priority data has been written to '" << filename << "'." << std::endl;    
}

void writeObstacleToYAML(const std::vector<std::vector<math::Vec2d>>& obstacle, const std::string& filename) {
    if (!isFileEmpty(filename)) {
        std::cout << "File '" << filename << "' is not empty. Not writing again." << std::endl;
        return;
    }

    // Create a YAML node representing the trajectory
    YAML::Emitter out;

    // 打开输出流
    std::ofstream file(filename);

    // 创建YAML::Node表示整个轨迹
    YAML::Node obs_;

    // 使用for循环写入多个轨迹点
    int i = 1;
    double t = 0.0;
    int obs_num = obstacle.size();
    for (const auto& obs : obstacle) {
        YAML::Node poseNode;
      for (int j = 0; j < obs.size(); j++) {
        poseNode["vx" + std::to_string(j+1)] = obs[j].x();
        poseNode["vy" + std::to_string(j+1)] = obs[j].y();
        // 将轨迹点节点添加到轨迹中
      }
      obs_["obstacle" + std::to_string(i++)] = poseNode;
    }

    // 将轨迹写入输出流
    out << obs_;

    // 将文档内容写入输出流
    file << out.c_str();

    // 关闭输出流
    file.close();

    std::cout << "Obetacle data has been written to '" << filename << "'." << std::endl;
}

std::vector<std::vector<math::Vec2d>> readObstacleFromYAML(const std::string& filename) {
    std::vector<std::vector<math::Vec2d>> obstacles;

    // 打开文件并加载 YAML 节点
    YAML::Node obs_ = YAML::LoadFile(filename);

    for (YAML::const_iterator it = obs_.begin(); it != obs_.end(); ++it) {
        std::vector<math::Vec2d> obstacle;
        YAML::Node poseNode = it->second;

        int j = 1;
        while (poseNode["vx" + std::to_string(j)] && poseNode["vy" + std::to_string(j)]) {
            double x = poseNode["vx" + std::to_string(j)].as<double>();
            double y = poseNode["vy" + std::to_string(j)].as<double>();
            obstacle.emplace_back(x, y);
            ++j;
        }

        obstacles.push_back(obstacle);
    }

    std::cout << "Obstacle data has been read from '" << filename << "'." << std::endl;
    return obstacles;
}

void writeTrajectoryToYAML_FG(const std::vector<std::vector<std::vector<double>>>& plan, const int numState, const std::string& filename) {
    // 打开输出流，使用 trunc 模式清空文件内容
    std::ofstream file(filename, std::ios::out | std::ios::trunc);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    // 创建 YAML Emitter
    YAML::Emitter out;

    // 创建 YAML::Node 表示整个轨迹
    YAML::Node traj_;

    // 使用 for 循环写入多个轨迹点
    int k = 1;
    for (int i = 0; i < plan.size(); i++) {
        for (int j = 0; j < numState; j++) {
            YAML::Node poseNode;
            poseNode["x"] = plan[i][j][0];
            poseNode["y"] = plan[i][j][1];
            poseNode["theta"] = plan[i][j][2];
            poseNode["v"] = plan[i][j][3];
            poseNode["omega"] = plan[i][j][4];
            poseNode["yaw"] = plan[i][j][5];
            poseNode["gimble_vel"] = plan[i][j][6];
            // 将轨迹点节点添加到轨迹中
            traj_["state" + std::to_string(k++)] = poseNode;
        }
    }

    // 将轨迹写入 YAML Emitter
    out << traj_;

    // 将 YAML 内容写入文件
    file << out.c_str();

    // 关闭文件流
    file.close();

    std::cout << "Trajectory data has been written to '" << filename << "'." << std::endl;
}

void writeTrajectoryToYAML(const FullStates& trajectory, const std::string& filename) {
    // if (!isFileEmpty(filename)) {
    //     std::cout << "File '" << filename << "' is not empty. Not writing again." << std::endl;
    //     return;
    // }

    // Create a YAML node representing the trajectory
    YAML::Emitter out;

    // 打开输出流
    std::ofstream file(filename);

    // 创建YAML::Node表示整个轨迹
    YAML::Node traj_;

    // 使用for循环写入多个轨迹点
    int i = 1;
    double tf = trajectory.tf;
    double t = 0.0;
    int states_num = trajectory.states.size();
    for (const auto& state : trajectory.states) {
        YAML::Node poseNode;
        poseNode["x"] = state.x;
        poseNode["y"] = state.y;
        poseNode["theta"] = state.theta;
        poseNode["phi"] = state.phi;
        poseNode["v"] = state.v;
        poseNode["omega"] = state.omega;
        poseNode["a"] = state.a;
        poseNode["t"] = t;
        t += tf / states_num;
        // 将轨迹点节点添加到轨迹中
        traj_["state" + std::to_string(i++)] = poseNode;
    }

    // 将轨迹写入输出流
    out << traj_;

    // 将文档内容写入输出流
    file << out.c_str();

    // 关闭输出流
    file.close();

    std::cout << "Trajectory data has been written to '" << filename << "'." << std::endl;
} 

void writeVectorToYAML(const std::vector<double>& data, const std::string& file_path) {
    try {
        // 创建一个新的 YAML 节点
        YAML::Node new_data;

        // 将向量内容添加到新节点中
        for (const auto& value : data) {
            new_data.push_back(value);
        }

        // 打开文件（使用 trunc 模式清空文件）并写入新数据
        std::ofstream file_out(file_path, std::ofstream::out | std::ofstream::trunc);
        if (!file_out) {
            throw std::ios_base::failure("Failed to open the file: " + file_path);
        }
        file_out << new_data;
        file_out.close();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}

void writeVectorsToYaml(const std::vector<std::vector<double>>& human_traj, const std::string& filename) {
    // 创建 YAML 节点
    YAML::Node node;

    // 将 human_traj 写入 YAML 节点
    for (const auto& traj : human_traj) {
        YAML::Node trajNode;
        for (const auto& value : traj) {
            trajNode.push_back(value);
        }
        node["human_traj"].push_back(trajNode);
    }

    // 使用 std::ofstream 清空文件并写入新的内容
    std::ofstream fout(filename, std::ios::out | std::ios::trunc); // 清空文件
    if (!fout.is_open()) {
        throw std::ios_base::failure("Failed to open the file: " + filename);
    }

    fout << node;
    fout.close();
}

std::vector<std::vector<double>> readVectorsFromYAML(const std::string& file_path) {
    std::vector<std::vector<double>> data;

    try {
        // 加载 YAML 文件
        YAML::Node yaml_data = YAML::LoadFile(file_path);

        // 访问 "human_traj" 节点
        if (!yaml_data["human_traj"]) {
            throw std::runtime_error("Missing 'human_traj' key in YAML file.");
        }

        YAML::Node traj_data = yaml_data["human_traj"];

        // 确认 "human_traj" 是序列
        if (!traj_data.IsSequence()) {
            throw std::runtime_error("'human_traj' is not a sequence.");
        }

        // 解析每个子节点
        for (const auto& node : traj_data) {
            if (node.IsSequence()) {
                std::vector<double> vec;
                vec.reserve(node.size());  // 提前分配内存
                for (const auto& element : node) {
                    vec.push_back(element.as<double>());
                }
                data.push_back(vec);
            } else {
                throw std::runtime_error("Expected sequence of sequences.");
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error reading YAML file: " << e.what() << std::endl;
    }

    return data;
}

std::vector<double> readVectorFromYAML(const std::string& file_path) {
    std::vector<double> data;
    try {
        // 打开文件并加载 YAML 内容
        std::ifstream file_in(file_path);
        if (!file_in) {
            throw std::ios_base::failure("Failed to open the file: " + file_path);
        }
        YAML::Node root = YAML::Load(file_in);

        // 检查根节点是否是序列
        if (root.IsSequence()) {
            // 遍历根节点中的所有值
            for (const auto& value : root) {
                data.push_back(value.as<double>());
            }
        } else if (root.IsMap()) {
            // 如果是 Map 节点，递归处理其中的所有序列
            for (auto it = root.begin(); it != root.end(); ++it) {
                if (it->second.IsSequence()) {
                    for (const auto& value : it->second) {
                        data.push_back(value.as<double>());
                    }
                } else {
                    throw std::runtime_error("Unexpected YAML format: Nested node is not a sequence.");
                }
            }
        } else {
            throw std::runtime_error("Unexpected YAML format: Root node is neither a sequence nor a map.");
        }

        file_in.close();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    return data;
}

void delete_yaml(const std::string& filename) {
  YAML::Node yaml_data = YAML::LoadFile(filename);
  std::ofstream fout(filename);
  fout << yaml_data;
  fout.close();
}

std::vector<std::vector<int>> generate_priority_set(std::string filename) {
    // 读取 YAML 文件
    YAML::Node yamlNode = YAML::LoadFile(filename);

    // 检查节点是否加载成功
    if (yamlNode.IsNull()) {
        std::cerr << "Failed to load YAML file." << std::endl;
    }

    // 从 YAML 节点中构造二维数组
    std::vector<std::vector<int>> priority_set_all;

    for (std::size_t i = 0; i < yamlNode.size(); ++i) {
    const YAML::Node& rowNode = yamlNode[i];
    std::vector<int> row;

    for (std::size_t j = 0; j < rowNode.size(); ++j) {
        const YAML::Node& valueNode = rowNode[j];
        row.push_back(valueNode.as<int>());
    }
    priority_set_all.push_back(row);
    }   
    return priority_set_all;
}

Trajectory_temp fulltotraj (const FullStates& full_traj) {
    Trajectory_temp traj;
    double tf = full_traj.tf;
    double t = 0.0;
    int states_num = full_traj.states.size();
    for (const auto& state : full_traj.states) {
        TrajectoryPointVector_temp vec;
        vec << state.x, state.y, state.theta, state.phi, state.v, state.omega, state.a, t;
        TrajectoryPoint_temp traj_wp(vec);
        traj.push_back(traj_wp);
        t += tf / states_num;
    }   
    return traj;
}

std::vector<std::vector<std::vector<double>>> generate_traj_fg(std::string filename, int numR) {  
    // 读取YAML文件
    YAML::Node traj_ = YAML::LoadFile(filename);

    // 检查文件是否成功加载
    if (!traj_.IsDefined()) {
        std::cerr << "Failed to load yaml file.\n";
    }
    int j = 0, k = 0;
    std::vector<std::vector<std::vector<double>>> plan(numR, std::vector<std::vector<double>>(traj_.size() / numR , std::vector<double>(7)));
    // 读取每个轨迹点
    for (const auto& kv : traj_) {
        const std::string& poseName = kv.first.as<std::string>();
        const YAML::Node& poseNode = kv.second;
        plan[j][k][0] = poseNode["x"].as<double>();
        plan[j][k][1] = poseNode["y"].as<double>();
        plan[j][k][2] = poseNode["theta"].as<double>();
        plan[j][k][3] = poseNode["v"].as<double>();
        plan[j][k][4] = poseNode["omega"].as<double>();
        plan[j][k][5] = poseNode["yaw"].as<double>();
        plan[j][k][6] = poseNode["gimble_vel"].as<double>();
        k++;
        if (k > traj_.size() / numR - 1) {
            j++;
            k = 0;
        }
    }
    return plan;
}

bool generate_traj_fg_(std::string filename, int numR, std::vector<std::vector<std::vector<double>>>& plan) {
    std::ifstream file(filename);
    if (!file.good()) {
        return false;
    }
  
    // 读取YAML文件
    YAML::Node traj_ = YAML::LoadFile(filename);

    // 检查文件是否成功加载
    if (!traj_.IsDefined()) {
        std::cerr << "Failed to load yaml file.\n";
    }
    int j = 0, k = 0;
    std::vector<std::vector<std::vector<double>>> plan_(numR, std::vector<std::vector<double>>(traj_.size() / numR , std::vector<double>(7)));
    // 读取每个轨迹点
    for (const auto& kv : traj_) {
        const std::string& poseName = kv.first.as<std::string>();
        const YAML::Node& poseNode = kv.second;
        plan_[j][k][0] = poseNode["x"].as<double>();
        plan_[j][k][1] = poseNode["y"].as<double>();
        plan_[j][k][2] = poseNode["theta"].as<double>();
        plan_[j][k][3] = poseNode["v"].as<double>();
        plan_[j][k][4] = poseNode["omega"].as<double>();
        plan_[j][k][5] = poseNode["yaw"].as<double>();
        plan_[j][k][6] = poseNode["gimble_vel"].as<double>();
        k++;
        if (k > traj_.size() / numR - 1) {
            j++;
            k = 0;
        }
    }
    plan = plan_;
    return true;
}

Trajectory_temp generate_traj(std::string filename) {
    Trajectory_temp traj;
    TrajectoryPoint_temp traj_point;
    // 读取YAML文件
    YAML::Node traj_ = YAML::LoadFile(filename);

    // 检查文件是否成功加载
    if (!traj_.IsDefined()) {
        std::cerr << "Failed to load yaml file.\n";
    }

    // 读取每个轨迹点
    for (const auto& kv : traj_) {
        const std::string& poseName = kv.first.as<std::string>();
        const YAML::Node& poseNode = kv.second;
        traj_point.x = poseNode["x"].as<double>();
        traj_point.y = poseNode["y"].as<double>();
        traj_point.theta = poseNode["theta"].as<double>();
        traj_point.phi = poseNode["phi"].as<double>();
        traj_point.v = poseNode["v"].as<double>();
        traj_point.omega = poseNode["omega"].as<double>();
        traj_point.a = poseNode["a"].as<double>();
        traj_point.t = poseNode["t"].as<double>();
        traj.push_back(traj_point);
    }
    return traj;
}

FullStates generate_ref_traj_diff(Trajectory_temp traj_lead, const double offset, const double angle) {
	Trajectory_temp traj_follower_diff; 
    FullStates ref_traj;
	traj_follower_diff.resize(traj_lead.size());
	for (size_t l_index = 0; l_index < traj_lead.size() - 1; l_index++) {
		traj_follower_diff[l_index].x = traj_lead[l_index].x + offset * cos(traj_lead[l_index].theta - angle);
		traj_follower_diff[l_index].y = traj_lead[l_index].y + offset * sin(traj_lead[l_index].theta - angle);
		traj_follower_diff[l_index].theta = traj_lead[l_index].theta;
    // traj_follower_diff[l_index].theta = atan2(traj_follower_diff[l_index + 1].y - traj_follower_diff[l_index].y, traj_follower_diff[l_index + 1].x - traj_follower_diff[l_index].x);
		traj_follower_diff[l_index].t = traj_lead[l_index].t;
	}
	for (size_t l_index = 0; l_index < traj_lead.size() - 1; l_index++) {
    double ds = hypot(traj_follower_diff[l_index + 1].x - traj_follower_diff[l_index].x, traj_follower_diff[l_index + 1].y - traj_follower_diff[l_index].y);
		traj_follower_diff[l_index].v = ds / (traj_follower_diff[l_index + 1].t - traj_follower_diff[l_index].t);
		traj_follower_diff[l_index].omega = (traj_lead[l_index + 1].theta - traj_lead[l_index].theta) / (traj_follower_diff[l_index + 1].t - traj_follower_diff[l_index].t);
    }
        for (size_t l_index = 0; l_index < traj_lead.size() - 1; l_index++) {
            traj_follower_diff[l_index].phi = (traj_lead[l_index + 1].omega - traj_lead[l_index].omega) / (traj_follower_diff[l_index + 1].t - traj_follower_diff[l_index].t);	
            traj_follower_diff[l_index].a = (traj_lead[l_index + 1].v - traj_lead[l_index].v) / (traj_follower_diff[l_index + 1].t - traj_follower_diff[l_index].t);
    }
    traj_follower_diff[traj_lead.size() - 1].x = traj_lead[traj_lead.size() - 1].x + offset * cos(traj_lead[traj_lead.size() - 1].theta - angle);
    traj_follower_diff[traj_lead.size() - 1].y = traj_lead[traj_lead.size() - 1].y + offset * sin(traj_lead[traj_lead.size() - 1].theta - angle);
    traj_follower_diff[traj_lead.size() - 1].theta = traj_lead[traj_lead.size() - 2].theta;
    traj_follower_diff[traj_lead.size() - 1].v = 0.0;
    traj_follower_diff[traj_lead.size() - 1].phi = 0.0;
    traj_follower_diff[traj_lead.size() - 1].a = 0.0;
    traj_follower_diff[traj_lead.size() - 1].omega = 0.0;
    traj_follower_diff[traj_lead.size() - 1].t = traj_lead[traj_lead.size() - 1].t;
    ref_traj.states.resize(traj_follower_diff.size());
    ref_traj.tf = traj_follower_diff[traj_follower_diff.size() - 1].t;
    for (int i_ref = 0; i_ref < traj_follower_diff.size(); i_ref++) {
        ref_traj.states[i_ref].x = traj_follower_diff[i_ref].x;
        ref_traj.states[i_ref].y = traj_follower_diff[i_ref].y;
        ref_traj.states[i_ref].theta = traj_follower_diff[i_ref].theta;
        ref_traj.states[i_ref].v = traj_follower_diff[i_ref].v;
        ref_traj.states[i_ref].phi = traj_follower_diff[i_ref].phi;
        ref_traj.states[i_ref].a = traj_follower_diff[i_ref].a;
        ref_traj.states[i_ref].omega = traj_follower_diff[i_ref].omega;
    }
    return ref_traj;
}

}