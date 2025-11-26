#pragma once

#include "robot_pai_controller/ControllerBase.h"
#include "robot_pai_controller/Types.h"

namespace robot_pai {

class PPOController : public ControllerBase {
    using tensor_element_t = float;

public:
    PPOController() = default;
    ~PPOController() override = default;

protected:
    bool loadModel(ros::NodeHandle& nh) override;
    bool loadRLCfg(ros::NodeHandle& nh) override;
    void computeActions() override;
    void computeObservation() override;
    void handleWalkMode() override;

private:
    // onnx policy model
    std::string policyFilePath_;
    std::shared_ptr<Ort::Env> onnxEnvPrt_;
    std::unique_ptr<Ort::Session> sessionPtr_;
    std::vector<const char*> inputNames_;
    std::vector<const char*> outputNames_;
    std::vector<Ort::AllocatedStringPtr> inputNodeNameAllocatedStrings;
    std::vector<Ort::AllocatedStringPtr> outputNodeNameAllocatedStrings;
    std::vector<std::vector<int64_t>> inputShapes_;
    std::vector<std::vector<int64_t>> outputShapes_;

    vector3_t baseLinVel_;
    vector3_t basePosition_;
    vector_t lastActions_;
    vector_t defaultJointAngles_;

    int actionsSize_;
    int observationSize_;
    std::vector<tensor_element_t> actions_;
    std::vector<tensor_element_t> observations_;
};


}