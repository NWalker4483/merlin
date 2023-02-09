#include "multi_axis.h"
#include "iostream"
#include <unistd.h>

#include <stdio.h>
#include <string>
#include <Eigen/Dense>

#include "pubSysCls.h"

#define ACC_LIM_RPM_PER_SEC 100000
#define VEL_LIM_RPM 700
#define TIME_TILL_TIMEOUT	10000	//The timeout used for homing(ms)

using namespace sFnd;
using namespace Eigen;
using namespace std;

class Lloyd : public MultiAxis<6>
{

private:
    SysManager *myMgr;
    INode *MotorNode[6];

    MatrixXd Degrees_Per_Step;
    MatrixXd Degrees_Per_Step_Inv;

public:
    Lloyd()
    {
        myMgr = SysManager::Instance();

        size_t portCount = 0;
        std::vector<std::string> comHubPorts;

        SysManager::FindComHubPorts(comHubPorts);
        printf("Found %d SC Hubs\n", (int)comHubPorts.size());
        
        for (portCount = 0; portCount < comHubPorts.size() && portCount < NET_CONTROLLER_MAX; portCount++)
        {
            myMgr->ComHubPort(portCount, comHubPorts[portCount].c_str()); // define the first SC Hub port (port 0) to be associated
                                                                          //  with COM portnum (as seen in device manager)
        }

        if (portCount < 1)
        {
            throw std::runtime_error("Unable to locate 2 SC hubs \n");
        }

        myMgr->PortsOpen(portCount);
        
        VectorXd MotorResolutions(6);

        for (size_t iPort = 0; iPort < portCount; iPort++)
        {
            IPort &myPort = myMgr->Ports(iPort);
            if (myPort.NodeCount() < 3)
            {
                // throw std::runtime_error("Unable to locate all nodes");
            }

            for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++)
            {
                INode &theNode = myPort.Nodes(iNode);

                theNode.EnableReq(false); // Ensure Node is disabled before loading config file
                // TODO: Confirm firmware version is 1.7.6
                
				myMgr->Delay(200);
                
                printf("   Node[%d]: type=%d\n", int(iNode), theNode.Info.NodeType());
				printf("            userID: %s\n", theNode.Info.UserID.Value());
				printf("        FW version: %s\n", theNode.Info.FirmwareVersion.Value());
				printf("          Serial #: %d\n", theNode.Info.SerialNumber.Value());
				printf("             Model: %s\n", theNode.Info.Model.Value());
				printf("        Resolution: %d\n", theNode.Info.PositioningResolution.Value());

				//The following statements will attempt to enable the node.  First,
				// any shutdowns or NodeStops are cleared, finally the node is enabled
				theNode.Status.AlertsClear();					//Clear Alerts on node 
				theNode.Motion.NodeStopClear();	// Clear Nodestops on Node  				
				theNode.Info.Ex.Parameter(98, 1); // Enable Interrupting Moves 
                theNode.EnableReq(true);					//Enable node 
				//At this point the node is enabled
				printf("Node \t%zi enabled\n", iNode);
				double timeout = myMgr->TimeStampMsec() + TIME_TILL_TIMEOUT;	//define a timeout in case the node is unable to enable
																			//This will loop checking on the Real time values of the node's Ready status
				while (!theNode.Motion.IsReady()) {
					if (myMgr->TimeStampMsec() > timeout) {
						printf("Error: Timed out waiting for Node %d to enable\n", iNode);

					}
				}

                theNode.Motion.MoveWentDone(); // Clear the rising edge Move done register
                
                theNode.AccUnit(INode::RPM_PER_SEC);           // Set the units for Acceleration to RPM/SEC
                theNode.VelUnit(INode::RPM);                   // Set the units for Velocity to RPM
                theNode.Motion.AccLimit = ACC_LIM_RPM_PER_SEC; // Set Acceleration Limit (RPM/Sec)
                theNode.Motion.VelLimit = VEL_LIM_RPM;         // Set Velocity Limit (RPM)
                // TODO Set torque limits 

                MotorNode[(iPort*3) + iNode] = &theNode;
            }
        }

        double reductions[36] = {
            1. / 48., 0, 0, 0, 0, 0,
            0, 1. / 48., 0, 0, 0, 0,
            0, -1. / 48., 1. / 48., 0, 0, 0,
            0, 0, 0, 1. / 24., 0, 0,
            0, 0, 0, -1. / 28.8, 1. / 28.8, 0,
            0, 0, 0, -1. / 12., 1. / 24., 1. / 24.};

        Degrees_Per_Step.resize(6,6);
        Degrees_Per_Step_Inv.resize(6,6);
        
        this->Degrees_Per_Step = Map<Matrix<double,6,6>>(reductions) * (360.L / 6400.L);
        this->Degrees_Per_Step_Inv = Degrees_Per_Step.inverse();
        
        this->setResolution(.5);
    }

    ~Lloyd()
    {
        // TODO: Add E Stop
        for (size_t i = 0; i < 2; i++)
        {
            IPort &myPort = myMgr->Ports(i);
            for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++)
            {
                // Create a shortcut reference for a node
                myPort.Nodes(iNode).EnableReq(false);
            }
        }
        // Close down the ports
        myMgr->PortsClose();
    }

    void delay(uint ms){
        myMgr->Delay(ms);
    }

protected:
    unsigned int getMillis()
    {
        return myMgr->TimeStampMsec();
    }
    
    void computeAxisPositions(double *axis_position)
    {
       VectorXd MotorPositions(6);

        for (size_t iNode = 0; iNode < 6; iNode++)
        {
            if (iNode > 0){
                MotorPositions(iNode) = 0;
                continue;
            }
            MotorNode[iNode]->Motion.PosnMeasured.Refresh();
            MotorPositions(iNode) = MotorNode[iNode]->Motion.PosnMeasured.Value();
        }

        VectorXd AxisPositions =  this->Degrees_Per_Step * MotorPositions;

        for (int i = 0; i < 6; i++)
        {
            *(axis_position + i) = AxisPositions(i);
        }
        cout << AxisPositions(0) << ", " << MotorPositions(0)<<endl;
    }

    void updateMotorSpeeds(double *axis_speeds)
    {
        VectorXd SetSpeeds =  Degrees_Per_Step_Inv * Map<Matrix<double,6,1>>(axis_speeds);
        for (size_t iNode = 0; iNode < 6; iNode++)
        {
            if (iNode > 0) continue;
            MotorNode[iNode]->Motion.MoveVelStart(SetSpeeds(iNode));
        }
    }
};
