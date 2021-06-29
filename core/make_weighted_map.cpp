#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <iostream>
#include <cmath> 
#include <vector>

/*
    DESCRIPTION: This code will take in arguments the name of a brute map
        in /map_brute/ folder and store in /data/map_weighted/ the weighted 
        map.
        And you will chose all interest point for your map, and store it
        in a .txt file.
*/

// STRUCTURE.
struct interest_point {
  int x;
  int y;
  std::string name;
} ;

// GLOBAL.
std::vector<interest_point> interest_points;

// CALLBACK.
void onMouse(int event, int x, int y, int flags, void *param)
{
    /*
        DESCRIPTION: when this callback is call, means that user have clic on 
            the map and can now add name to this area. 
        INFORMATION: you need to choose un empty area.
    */

	cv::Mat *im = reinterpret_cast<cv::Mat*>(param);
	switch (event){
	case cv::EVENT_LBUTTONDOWN:

        if(static_cast<int>(im->at<uchar>(cv::Point(x, y))) == 255)
        {   
            std::cout << "[NEWPT] Write a name of interest point in lower case and press 'entrer' : ";

            // GET NAME.
            std::string name_of_interest_point;
            std::getline(std::cin, name_of_interest_point);

            // ADD NEW INTEREST POINT.
            interest_point new_point;
            new_point.x    = x;
            new_point.y    = y;
            new_point.name = name_of_interest_point;
            interest_points.push_back(new_point);

            std::cout << "[NEWPT] New point added.\n";
        }
        else
        {
            std::cout << "[ERROR] You need to select an empty area.\n";
        }

		break;
	}
}

// FONCTION.
double distance(int A_i, int A_j, int B_i, int B_j)
{
    return pow(pow(A_i-B_i,2)+(pow(A_j-B_j,2)),0.5);
}

bool save_map(cv::Mat map_weighted, std::string name)
{
    // DESCRIPTION: save the weighted map.
    std::string directory = "../data/map_weighted/";
    std::string path_weighted_map = directory.append(name).append(".png");

    return cv::imwrite(path_weighted_map, map_weighted);
}

bool save_point(std::string name)
{
    // DESCRIPTION: save all points in .txt file.
    std::string directory = "../data/map_interest_point/";
    std::string path_interest_point = directory.append(name).append("_weighted.txt");

    std::ofstream myfile;
    myfile.open(path_interest_point);

    for(auto interest_point : interest_points)
    {
        myfile << interest_point.x << "/" << interest_point.y << "/" << interest_point.name << "/\n";
    }
    myfile.close();
    return true;
}

// MAIN.
int main(int argc, char** argv)
{
    std::string directory = "../data/map_brute/";
    std::string path_brute_map = directory.append(argv[1]).append(".png");
    std::cout << "Brute map from : " << path_brute_map << "\n";

    std::string image_path = cv::samples::findFile(path_brute_map);
    cv::Mat img = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    if(img.empty())
    {
        std::cout << "Could not read the image: " << image_path << std::endl;
        return 1;
    }

    cv::imshow("Display window", img);
    int k = cv::waitKey(0); 

    /*
        PROCESS 1: first we will put all gray scale in black and white color
            will stay in white.
        INFO     : 
            > Empty space   = 255	case CV_EVENT_LBUTTONDOWN:

            > Fullfil space = 0
    */

    std::cout << "[START] Process 1 : Clean all map. \n";
    for(int i=0; i<img.rows; i++)
        for(int j=0; j<img.cols; j++)
            if(img.at<uchar>(i,j) != 255 && img.at<uchar>(i,j) != 0) img.at<uchar>(i,j) = 0;
    std::cout << "  [END] Process 1. \n";

    cv::imshow("Display window", img);
    int k2 = cv::waitKey(0); 

    /*  PROCESS 2: we will generate the gray scale map that will represente
            the weighted pixel.
        LOGIC    : we have 3 layer of protection and it's not a continue 
            method.
            > 0. Obstacle                  =  0 cm
            > 1. Impossible physiquement   =  0 cm < 30 cm = (color value = 50)
            > 2. Interdie sécurité drift   = 30 cm < 50 cm = (color value = 120)
            > 3. Possible mais deconseillé = 50 cm < 70 cm = (color value = 200)
            > X. Free space                = 70 cm < ++ cm
    */

    std::cout << "[START] Process 2 : Define all area. \n";
    int physic_border        = 6;  // 5 cm x 6 = 30 cm
    int drift_security       = 9;  // 6 + (5 cm x 3) 
    int try_avoid            = 13;

    int color_physic_border  = 50;
    int color_drift_security = 120;
    int color_try_avoid      = 200;
    int new_color            = -1;

    for(int i=0; i<img.rows; i++)
        for(int j=0; j<img.cols; j++)
            if(img.at<uchar>(i,j) == 255)
            {
                bool found_obstacle       = false;
                double distance_obstacle  = 1;
                int obstacle_i            = -1;
                int obstacle_j            = -1;
                int memory                = try_avoid;      //special variable to stop last found good effect.

                while(!found_obstacle && distance_obstacle <= try_avoid)
                {   
                    // run segment A. (ligne du haut)
                    for(int ii = -distance_obstacle; ii<= distance_obstacle; ii++)
                        if((ii+i >= 0 && ii+i < img.rows) && (j-distance_obstacle >= 0 && j-distance_obstacle < img.cols))
                            if(img.at<uchar>(i+ii,j+distance_obstacle) == 0) 
                            {
                                if(memory > pow(pow(ii,2),0.5))
                                    obstacle_i = i+ii; obstacle_j = j+distance_obstacle;
                                    found_obstacle = true;
                            }
                    if(found_obstacle) break;

                    // run segment B. (ligne du bas)
                    for(int ii = -distance_obstacle; ii<= distance_obstacle; ii++)
                        if((ii+i >= 0 && ii+i < img.rows) && (j-distance_obstacle >= 0 && j-distance_obstacle < img.cols))
                            if(img.at<uchar>(i+ii,j-distance_obstacle) == 0)
                            {   
                                if(memory > pow(pow(ii,2),0.5))
                                    obstacle_i = i+ii; obstacle_j = j-distance_obstacle;
                                    found_obstacle = true;
                            }
                    if(found_obstacle) break;

                    // run segment C. (ligne de droite)
                    for(int jj = -distance_obstacle; jj<= distance_obstacle; jj++)
                        if((jj+j >= 0 && jj+j < img.cols) && (i-distance_obstacle >= 0 && i-distance_obstacle < img.rows))
                            if(img.at<uchar>(i+distance_obstacle,j+jj) == 0) 
                            {   
                                if(memory > pow(pow(jj,2),0.5))
                                    obstacle_i = i+distance_obstacle; obstacle_j = j+jj;
                                    found_obstacle = true;
                            }
                    if(found_obstacle) break;

                    // run segment D. (ligne de gauche)
                    for(int jj = -distance_obstacle; jj<= distance_obstacle; jj++)
                        if((jj+j >= 0 && jj+j < img.cols) && (i-distance_obstacle >= 0 && i-distance_obstacle < img.rows))
                            if(img.at<uchar>(i-distance_obstacle,j+jj) == 0) 
                            {
                                if(memory > pow(pow(jj,2),0.5))
                                    obstacle_i = i-distance_obstacle; obstacle_j = j+jj;
                                    found_obstacle = true;
                            }
                    if(found_obstacle) break;

                    distance_obstacle += 1;
                }
                distance_obstacle = distance(i, j, obstacle_i, obstacle_j);

                if(     distance_obstacle > 0              && distance_obstacle <= physic_border ) new_color = color_physic_border;
                else if(distance_obstacle > physic_border  && distance_obstacle <= drift_security) new_color = color_drift_security;
                else if(distance_obstacle > drift_security && distance_obstacle <= try_avoid     ) new_color = color_try_avoid;
                else                                                                               new_color = 255;
                img.at<uchar>(i,j) = new_color;
            }

    std::cout << "  [END] Process 2. \n";
    cv::imshow("Display window", img);
    int k3 = cv::waitKey(0); 

    /*  PROCESS 3 : let's user select point of interest and give him name.
        DATA      : 
        > interest_points = vector of structure interest_point.
    */
    std::cout << "[START] Process 3 : Define new points on map. \n";
    cv::setMouseCallback("Display window", onMouse, reinterpret_cast<void *>(&img));
    int k4 = cv::waitKey(0); 
    std::cout << "  [END] Process 3. \n";

    /*  PROCESS 4 : save all, new weighted map and all points.
    */
    if(save_map(img, argv[1])){ std::cout << "[SAVED] New weighted map saved.\n";}
    if(save_point(argv[1]))   { std::cout << "[SAVED] New interest points saved.\n";}

    return 0;
}