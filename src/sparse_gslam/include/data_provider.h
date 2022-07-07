#include <g2o/types/slam2d/se2.h>

#include <memory>
#include <string>
#include <vector>
class DataProvider {
   public:
    virtual bool get_data(double& time, g2o::SE2& pose, std::vector<float>& full_range) = 0;
    virtual ~DataProvider() {}
};

std::unique_ptr<DataProvider> create_data_provider(const std::string& name, const std::string& fpath);