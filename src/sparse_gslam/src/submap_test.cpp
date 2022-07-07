#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "cartographer_bindings/range_data_inserter_2d.h"
#include "submap.h"
#include "ros/ros.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "ls_extractor/matplotlibcpp.h"

//!pwd/pwd/18.csv, pwd/pwd/match44_18.csv, true, score 0.420748, new score 0.49039
// pwd/pwd/11.csv, pwd/pwd/match11_11.csv, false, score 0.412863, new score 0.708178
//!pwd/pwd/94.csv, pwd/pwd/match36_94.csv, true, score 0.428638, new score 0.573387
// pwd/pwd/44.csv, pwd/pwd/match33_44.csv, false, score 0.463311, new score 0.721469
// pwd/pwd/15.csv, pwd/pwd/match13_15.csv, false, score 0.400475, new score 0.660784
// pwd/pwd/5.csv, pwd/pwd/match20_5.csv, false, score 0.400475, new score 0.63683

std::string fn = "pwd/pwd/18.csv";
std::string fn2 = "pwd/pwd/match44_18.csv";
bool reverse = true; 

namespace plt = matplotlibcpp;
void construct_submap(){
    std::fstream reader;
    reader.open(fn, std::ios::in);
    std::string line, word;
    sensor::RangeData2D temp_range;

    std::getline(reader, line);
    std::stringstream s(line);
    std::getline(s, word, ',');
    int mid = std::stoi(word);

    std::vector<double> pose_est;
    std::getline(s, word, ',');
    pose_est.push_back(std::stod(word));
    std::getline(s, word, ',');
    pose_est.push_back(std::stod(word));
    std::getline(s, word, ',');
    pose_est.push_back(std::stod(word));
    g2o::VertexSE2 pose;
    pose.setEstimateData(pose_est);


    while(!reader.eof()){
        std::getline(reader, line);
        std::stringstream s(line);

        std::getline(s, word, ',');
        if(word == ""){
            break;
        }
        float x = std::stof(word);
        std::getline(s, word, ',');
        float y = std::stof(word);

        temp_range.points_.push_back(Eigen::Vector2f(x,y));
    }
    while(!reader.eof()){
        std::getline(reader, line);
        std::stringstream s(line);

        std::getline(s, word, ',');
        if(word == ""){
            break;
        }
        int start = std::stoi(word);
        std::getline(s, word, ',');
        int end = std::stoi(word);
        std::getline(s, word, ',');
        int return_end = std::stoi(word);
        std::getline(s, word, ',');
        float originx = std::stof(word);
        std::getline(s, word, ',');
        float originy = std::stof(word);

        temp_range.meta_data_.push_back({start, return_end, end, Eigen::Vector2f(originx,originy)});
    }
    std::cout << temp_range.points_.size() << std::endl;
    std::cout << temp_range.meta_data_.size() << std::endl;
    Submap submap(1, 0.1);
    mapping::MultirangeDataInserter range_data_inserter(0.8,0.2);
    range_data_inserter.Insert(temp_range, &submap.probability_grid);
    submap.probability_grid = std::move(*static_cast<mapping::ProbabilityGrid*>(submap.probability_grid.ComputeCroppedGrid().release()));
    mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions2D matcher_options;
    matcher_options.set_angular_search_window(0.7);
    matcher_options.set_linear_search_window(5.0);
    matcher_options.set_branch_and_bound_depth(6);
    submap.fix_submap(&pose, matcher_options, 1.0f);

    submap.range = std::move(temp_range);

    // get scan

    std::fstream reader2;
    reader2.open(fn2, std::ios::in);

    sensor::PointCloud c_pc;
    while(!reader2.eof()){
        std::getline(reader2, line);
        std::stringstream s(line);

        std::getline(s, word, ',');
        if(word == ""){
            break;
        }
        float x = std::stof(word);
        std::getline(s, word, ',');
        float y = std::stof(word);
        std::getline(s, word, ',');

        if(reverse){
            c_pc.push_back({{-x,-y,0.0f}});
        }else{
            c_pc.push_back({{x,y,0.0f}});
        }
    }

    // match 
    float score = 0;
    transform::Rigid2d pose_estimate;
    submap.matcher.get().Match(transform::Rigid2d::Identity(), c_pc, 0.4, &score, &pose_estimate);
    std::cout << "score" << score << std::endl;

    std::cout << pose.estimate().toVector() << std::endl;

    // analysis
    std::cout << "c_pc size: " << c_pc.size() << std::endl;
    // for(auto it = submap.matcher.get().m.begin(); it != submap.matcher.get().m.end(); it++){
    //     std::cout << it->first << " : " << it->second << std::endl;
    // }

    // visualize
    std::vector<float> x, y;
    x.reserve(c_pc.size());
    y.reserve(c_pc.size());
    auto _pef = pose_estimate.cast<float>();
    for (auto& p : c_pc) {
        Eigen::Vector2f pi = (_pef * p.position.head<2>() - submap.box.min()) / submap.grid.info.resolution;
        x.push_back(pi[0]);
        y.push_back(pi[1]);
    }
    auto& grid = submap.grid;
    plt::clf();
    plt::imshow((unsigned char*)grid.data.data(), grid.info.height, grid.info.width, 1, {{"cmap", "gray"}});
    plt::scatter(x, y, 5.0);
    plt::axis("equal");
    plt::show(); 
}


int main(int argc, char **argv){
    construct_submap();
    return 0;
} 