#include <ros/ros.h>
#include "ls_extractor/defs.h"

namespace ls_extractor {

void parseParams(Params& params, const XmlRpc::XmlRpcValue& config) {
    params.max_line_gap = (double)config["max_line_gap"];
    params.min_line_length = (double)config["min_line_length"];
    params.min_line_points = config["min_line_points"];
    params.cluster_threshold = (double)config["cluster_threshold"];
    params.min_split_dist = (double)config["min_split_dist"];
    params.outlier_dist = (double)config["outlier_dist"];
    params.rmse_thresh = (double)config["rmse_thresh"];
}

}