#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <cmath> 

/*
    DESCRIPTION: This code will take in arguments the path of a brute map
        and store in /data/map_weighted/ the weighted map.
*/

double distance(int A_i, int A_j, int B_i, int B_j)
{
    return pow(pow(A_i-B_i,2)+(pow(A_j-B_j,2)),0.5);
}

int main(int argc, char** argv)
{
    std::string path_brute_map = argv[1];
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
            > Empty space   = 255
            > Fullfil space = 0
    */

    std::cout << "[START] Process 1. \n";
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

    std::cout << "[START] Process 2. \n";
    int physic_border        = 6;  // 5 cm x 6 = 30 cm
    int drift_security       = 9;  // 6 + (5 cm x 4) 
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

    return 0;
}