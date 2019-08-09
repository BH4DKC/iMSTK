#include "imstkToolSurfaceInteractionPair.h"
#include "imstkPbdObject.h"
#include <g3log/g3log.hpp>

namespace imstk
{
//Predefined name strings for the cutter and grasper
std::string cutterStr = "cutterObject";
std::string grasperStr = "grasperObject";
void ToolSurfaceInteractionPair::updateTools()
{
    //Must use the specified name strings to get tools registered
    if (m_toolClient && (m_objects.first->getName() == cutterStr || m_objects.first->getName() == grasperStr
        || m_objects.second->getName() == cutterStr || m_objects.second->getName() == grasperStr) ) 
    {
        //adding tris collided with tool to the ToolCollisionData -- disabled for now
        /*for (auto data : m_colData->TVColData) 
        {
            m_toolState->triId.insert(data.triIdA);
        }*/

        float bladeLength = 1.2; //assume the blade length is 1.2
        //Update toolState
        //first obj is surface
        if (m_objects.first->getName() != cutterStr && m_objects.first->getName() != grasperStr) 
        {
            auto obj = std::dynamic_pointer_cast<PbdObject> (m_objects.first);
            auto toolobj = std::dynamic_pointer_cast<PbdObject> (m_objects.second);

            auto geo = toolobj->getPhysicsGeometry();
            if (auto mesh = std::dynamic_pointer_cast<PointSet>(geo))
            {
                //toolDir is the unit vector that defines the current tool orientation
                Vec3d toolDir = mesh->getRotation() * Vec3d(0, 0, -1);
                m_toolState->cutPlaneNormal = mesh->getRotation() * Vec3d(0, 1, 0);
                m_toolState->toolTipStartPos = mesh->getTranslation(); //tip position, based on the assumption that the tooltip is placed at the origin in the model
                m_toolState->toolTipEndPos = bladeLength *toolDir + m_toolState->toolTipStartPos;
                m_toolState->toolDir = toolDir;
                switch (m_toolState->toolFunction)
                {
                case ToolState::ToolFunction::CUT:
                    if (!m_newCutHappened && m_toolClient->getButton(1) && m_lapController->getJawAngle() < PI / 18) //cut happens when the jaw is closing and the angle is < 20'
                    {
                        obj->generateCut(m_toolState);
                        m_toolState->button2Pressed = true;
                        m_newCutHappened = true;
                    }
                    break;

                case ToolState::ToolFunction::GRASP:
                    if (m_toolClient->getButton(1) && !m_toolState->isDoingGrasp)
                    {
                        std::cout << "doing grasp... \n";
                        m_toolState->button2Pressed = m_toolClient->getButton(1);
                        if(obj->doGrasp(m_toolState))
                            m_toolState->isDoingGrasp = true;
                    }
                    break;
                }
            }
        }
        else //second is surface
        {
            auto obj = std::dynamic_pointer_cast<PbdObject> (m_objects.second);
            auto toolobj = std::dynamic_pointer_cast<PbdObject> (m_objects.first);

            auto geo = toolobj->getPhysicsGeometry();
            if (auto mesh = std::dynamic_pointer_cast<PointSet>(geo))
            {
                //toolDir is the unit vector that defines the current tool orientation
                Vec3d toolDir = mesh->getRotation() * Vec3d(0, 0, -1);
                //cutPlaneNormal is the unit vector that is orthogonal to the cutting plane
                m_toolState->cutPlaneNormal = mesh->getRotation() * Vec3d(0, 1, 0);
                m_toolState->toolTipStartPos = mesh->getTranslation(); //tip position, based on the assumption that the tooltip is placed at the origin in the model
                m_toolState->toolTipEndPos = bladeLength *toolDir + mesh->getTranslation(); 
                m_toolState->toolDir = toolDir;
                switch (m_toolState->toolFunction)
                {
                case ToolState::ToolFunction::CUT:
                    if (!m_newCutHappened && m_toolClient->getButton(1) && m_lapController->getJawAngle() < PI / 18) //cut happens when the jaw is closing and the angle is < 20'
                    {
                        obj->generateCut(m_toolState);
                        m_toolState->button2Pressed = true;
                        m_newCutHappened = true;
                    }
                    break;

                case ToolState::ToolFunction::GRASP:
                    if (m_toolClient->getButton(1) && !m_toolState->isDoingGrasp)
                    {
                        std::cout << "doing grasp... \n";
                        m_toolState->button2Pressed = m_toolClient->getButton(1);
                        if (obj->doGrasp(m_toolState))
                            m_toolState->isDoingGrasp = true;
                    }
                    break;
                }
            }
        }
        //reset tool contact data
        /*if (m_toolState->triId.size() > 0) 
        {
            m_toolState->clearData();
        }*/

        //Reset the cutting or grasping
        switch (m_toolState->toolFunction)
        {
        case ToolState::ToolFunction::CUT:
            //Reset the cut flag when the jaw angle is > 30'
            if (m_newCutHappened && m_lapController->getJawAngle() > PI / 12) 
            {
                m_newCutHappened = false;
            }
            break;

        case ToolState::ToolFunction::GRASP:
            //Release the grasp when the 1st button is pressed
            if (m_toolClient->getButton(0) && !m_toolClient->getButton(1) && m_toolState->isDoingGrasp)
            {
                auto obj = std::dynamic_pointer_cast<PbdObject> (m_objects.first);
                if (m_objects.first->getName() == grasperStr)
                    obj = std::dynamic_pointer_cast<PbdObject> (m_objects.second);
                std::cout << "doing ungrasp... \n";
                m_toolState->button2Pressed = m_toolClient->getButton(1);
                obj->unGrasp(m_toolState);
                m_toolState->isDoingGrasp = false;
            }
            break;
        }

        //Reset buttonPressed to false
        if (!m_toolClient->getButton(0) && m_toolState->button1Pressed)
        {
            m_toolState->button1Pressed = false;
        }
        if (!m_toolClient->getButton(1) && m_toolState->button2Pressed)
        {
            m_toolState->button2Pressed = false;
        }

    }
}











}