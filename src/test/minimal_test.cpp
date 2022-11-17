#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;

#include <sbpl/headers.h>

int planxythetalat(bool forwardSearch = true)
{
    double allocated_time_secs = 10.0; // in seconds

    MDPConfig MDPCfg;



    // set the perimeter of the robot (it is given with 0,0,0 robot ref. point for which planning is done)
    vector<sbpl_2Dpt_t> perimeterptsV; {
        sbpl_2Dpt_t pt_m;
        double halfwidth = 5.; //0.3;
        double halflength = 10.; //0.45;
        pt_m.x = -halflength;
        pt_m.y = -halfwidth;
        perimeterptsV.push_back(pt_m);
        pt_m.x = halflength;
        pt_m.y = -halfwidth;
        perimeterptsV.push_back(pt_m);
        pt_m.x = halflength;
        pt_m.y = halfwidth;
        perimeterptsV.push_back(pt_m);
        pt_m.x = -halflength;
        pt_m.y = halfwidth;
        perimeterptsV.push_back(pt_m);
        //perimeterptsV.clear();
    }

    cv::Mat map(128, 128, CV_8UC1, cv::Scalar(0));
    cv::circle(map, cv::Point(70, 25), 20, cv::Scalar(20), 5, cv::LINE_AA);
    cv::circle(map, cv::Point(50, 75), 25, cv::Scalar(100), 5, cv::LINE_AA);


    // Initialize Environment (should be called before initializing anything else)
    EnvironmentNAVXYTHETALAT environment_navxythetalat;

    bool r =
        //environment_navxythetalat.InitializeEnv("../env_examples/nav3d/env1.cfg", perimeterptsV, "../matlab/mprim/pr2.mprim");
        environment_navxythetalat.InitializeEnv(
            128, 128,
            map.data,
            11, 10, 0.2,
            90, 82, 2.,
            20., 20., 2., //goal tolerance is ignored
            perimeterptsV,
            1.0,
            1.3,
            9999.,
            50,
            "../test.mprim"
        );
    if (!r) {
        throw SBPL_Exception("ERROR: InitializeEnv failed");
    }

    // Initialize MDP Info
    if (!environment_navxythetalat.InitializeMDPCfg(&MDPCfg)) {
        throw SBPL_Exception("ERROR: InitializeMDPCfg failed");
    }



    bool bforwardsearch = true;
    SBPLPlanner* planner =
                      new ARAPlanner(&environment_navxythetalat, bforwardsearch);
                      //new ADPlanner(&environment_navxythetalat, bforwardsearch);

    // set planner properties
    if (planner->set_start(MDPCfg.startstateid) == 0) {
        printf("ERROR: failed to set start state\n");
        throw SBPL_Exception("ERROR: failed to set start state");
    }
    if (planner->set_goal(MDPCfg.goalstateid) == 0) {
        printf("ERROR: failed to set goal state\n");
        throw SBPL_Exception("ERROR: failed to set goal state");
    }

    double initialEpsilon = 3.;
    bool bsearchuntilfirstsolution = true;
    planner->set_initialsolution_eps(initialEpsilon);
    planner->set_search_mode(bsearchuntilfirstsolution);

    // plan
    printf("start planning...\n");
    vector<int> solution_stateIDs_V;
    int bRet = planner->replan(allocated_time_secs, &solution_stateIDs_V);
    printf("done planning\n");
    printf("size of solution=%d\n", (unsigned int)solution_stateIDs_V.size());

    environment_navxythetalat.PrintTimeStat(stdout);

    //print solution
    printf("Solution:\n================\n");
    for (size_t i = 0; i < solution_stateIDs_V.size(); i++) {
        int x;
        int y;
        int theta;
        environment_navxythetalat.GetCoordFromState(solution_stateIDs_V[i], x, y, theta);
        printf("%d: %d %d %d\t\t%.3f %.3f %.3f\n", solution_stateIDs_V[i],
          x, y, theta,
          DISCXY2CONT(x, 0.025), DISCXY2CONT(y, 0.025), DiscTheta2Cont(theta, 16)
        );

        map.at<uint8_t>(y, x) = 255;
    }

    // print the continuous solution
    printf("Continuous Solution:\n================\n");
    vector<sbpl_xy_theta_pt_t> xythetaPath;
    environment_navxythetalat.ConvertStateIDPathintoXYThetaPath(&solution_stateIDs_V, &xythetaPath);
    printf("solution size=%d\n", (unsigned int)xythetaPath.size());
    for (unsigned int i = 0; i < xythetaPath.size(); i++) {
        printf("%.3f %.3f %.3f\n", xythetaPath.at(i).x, xythetaPath.at(i).y, xythetaPath.at(i).theta);


    }

    environment_navxythetalat.PrintTimeStat(stdout);

    cv::imshow("test", map);
    cv::waitKey(0);
    delete planner;
    return bRet;
}

int main(int argc, char *argv[])
{
    int plannerRes = planxythetalat();
    if (plannerRes == 1) {
        printf("Solution is found\n");
    }
    return 0;
}
