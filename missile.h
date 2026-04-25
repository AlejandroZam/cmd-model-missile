#pragma once
#include "block.h"
#include "dds_sub.h"
#include "noise.h"
#include <Eigen/Dense>
#include <string>
#include <vector>

// Point-mass missile with proportional navigation (PN) guidance.
// States: 3-D position and velocity (6 integrators).
// Subscribes to target state on DDS topic "sim.<target_topic>.state".
// topic name is set via model YAML field: target_topic
class Missile : public Block {
public:
    Missile();

    void loadConfig(const std::string& path) override;
    void seed(uint64_t s) override;
    void initialize() override;
    void eventUpdate() override;
    void derivatives() override;
    void report() override;

    const Eigen::Vector3d& pos3() const { return pos_; }
    const Eigen::Vector3d& vel3() const { return vel_; }

private:
    Eigen::Vector3d pos_  = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel_  = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_  = Eigen::Vector3d::Zero();
    Eigen::Vector3d pos0_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel0_ = Eigen::Vector3d::Zero();

    Eigen::Vector3d tpos_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d tvel_ = Eigen::Vector3d::Zero();
    bool            hasTargetData_ = false;
    Eigen::Vector3d noiseAcc_      = Eigen::Vector3d::Zero();

    double   range_     = 0.0;
    double   navRatio_  = 4.0;
    double   aMax_      = 200.0;
    double   missDist_  = 20.0;
    double   reportDt_  = 1.0;
    bool     intercept_ = false;

    std::string  targetTopic_;
    SimSubscriber subscriber_;
    Logger        logger_;
    NoiseGen      noiseAx_, noiseAy_, noiseAz_;
    std::vector<std::string> outputSignals_;
};
