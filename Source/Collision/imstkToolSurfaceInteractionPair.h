#pragma once

#include "imstkInteractionPair.h"
#include "imstkLaparoscopicToolController.h"
#include <set>
namespace imstk
{

/// ToolState stores: tool state, contact
struct ToolState
{
    enum class ToolFunction
    {
        CUT,
        GRASP
    };

    //Tool Contact Data
    std::vector<size_t> nodeId;
    std::set<size_t> triId;

    //Cutting tool position/orientation
    Vec3d toolDir; //unit dir along the shaft
    Vec3d cutPlaneNormal; // unit normal vecotr of the cutting plane 
    Vec3d toolTipStartPos, toolTipEndPos; //start point and the end point defines the cutting blade
    ToolFunction toolFunction = ToolFunction::CUT; //default tool function CUT
        
    //Tool Button State
    bool button1Pressed = false;
    bool button2Pressed = false;
    bool isDoingGrasp = false;

    ToolState() {
        nodeId.resize(0);
        nodeId.reserve(10);
        triId.clear();
        toolTipStartPos = Vec3d(0, 0, 0);
        toolTipEndPos = Vec3d(0, 0, 0);
    }

    void clearData() //Clear Collision Data
    {
        nodeId.resize(0);
        triId.clear();
    }
};


class ToolSurfaceInteractionPair : public InteractionPair
{
    using ObjectsPair = std::pair<std::shared_ptr<CollidingObject>, std::shared_ptr<CollidingObject>>;

public:

    ///
    /// \brief Constructor inherited from InteractionPair
    ///
    ToolSurfaceInteractionPair(std::shared_ptr<CollidingObject> A,
        std::shared_ptr<CollidingObject> B,
        CollisionDetection::Type CDType,
        CollisionHandling::Type CHAType,
        CollisionHandling::Type CHBType) :InteractionPair(A, B, CDType, CHAType, CHBType) {}

    ToolSurfaceInteractionPair(std::shared_ptr<CollidingObject> A,
        std::shared_ptr<CollidingObject> B,
        std::shared_ptr<CollisionDetection> CD,
        std::shared_ptr<CollisionHandling> CHA,
        std::shared_ptr<CollisionHandling> CHB) :
        InteractionPair(A, B, CD, CHA, CHB) {
        m_toolState = std::make_shared<ToolState>();
    }


    ///
    /// \brief Destructor
    ///
    ~ToolSurfaceInteractionPair() {
        m_toolState->clearData();
    }
    
    //update tool states and apply tool functions
    void updateTools();

    inline void setDeviceClient(std::shared_ptr<DeviceClient> client) { m_toolClient = client; }

    inline void setLapToolController(std::shared_ptr<LaparoscopicToolController> controller) { m_lapController = controller; }

    inline void setToolFunction(ToolState::ToolFunction function) { m_toolState->toolFunction = function; }

    inline std::shared_ptr<ToolState> getToolState() { return m_toolState; }

protected:

    std::shared_ptr<ToolState> m_toolState;
    std::shared_ptr<DeviceClient> m_toolClient;       
    std::shared_ptr<LaparoscopicToolController> m_lapController;
    bool m_newCutHappened = false; ///< make sure the cut is generated one by one


};







}