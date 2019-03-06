#include "../PerseusLib/PerseusLib.h"

#include "Utils/Timer.h"
#include <opencv2/opencv.hpp>

using namespace Perseus::Utils;
int main_single(void);
int main_iter(char* mask_format, char* pose_format, int current_round);
int main(int argc, char *argv[])
{
    if(argc == 1){
        return main_single();
    }else if(argc < 4){
        std::cout << "miss input param" << std::endl;
        return -1;
    }else{
        return main_iter(argv[1], argv[2], atoi(argv[3]));
    }
}

int main_single(void)
{
    std::string sModelPath =      "/home/rlvc/Workspace/0_code/PWP3D/Files/Models/Renderer/long.obj";
    std::string sSrcImage =       "/home/rlvc/Workspace/0_code/PWP3D/Files/Images/Red.png";
    std::string sCameraMatrix =   "/home/rlvc/Workspace/0_code/PWP3D/Files/CameraCalibration/900nc.cal";
    std::string sTargetMask =     "/home/rlvc/Workspace/0_code/PWP3D/Files/Masks/480p_All_VideoMask.png";
    std::string sHistSrc =        "/home/rlvc/Workspace/0_code/PWP3D/Files/Masks/Red_Source.png";
    std::string sHistMask =       "/home/rlvc/Workspace/0_code/PWP3D/Files/Masks/Red_Mask.png";


  // blue car demo
  //  std::string sModelPath = "/Users/luma/Code/DataSet/Mesh/BlueCar.obj";
  //  std::string sSrcImage = "/Users/luma/Code/Luma/PWP3D/Files/Images/248-LiveRGB.png";
  //  std::string sCameraMatrix = "/Users/luma/Code/Luma/PWP3D/Files/CameraCalibration/Kinect.cal";
  //  std::string sTargetMask = "/Users/luma/Code/Luma/PWP3D/Files/Masks/480p_All_VideoMask.png";
  //  std::string sHistSrc = "/Users/luma/Code/Luma/PWP3D/Files/Images/248-LiveRGB.png";
  //  std::string sHistMask = "/Users/luma/Code/Luma/PWP3D/Files/Masks/248-ID-3-LiveImage.png";

//  // red can demo
//  std::string sModelPath = "/Users/luma/Code/DataSet/Mesh/RedCan.obj";
//  std::string sSrcImage = "/Users/luma/Code/Luma/PWP3D/Files/Images/248-LiveRGB.png";
//  std::string sCameraMatrix = "/Users/luma/Code/Luma/PWP3D/Files/CameraCalibration/Kinect.cal";
//  std::string sTargetMask = "/Users/luma/Code/Luma/PWP3D/Files/Masks/480p_All_VideoMask.png";
//  std::string sHistSrc = "/Users/luma/Code/Luma/PWP3D/Files/Images/248-LiveRGB.png";
//  std::string sHistMask = "/Users/luma/Code/Luma/PWP3D/Files/Masks/248-ID-1-LiveImage.png";

  // ---------------------------------------------------------------------------
  char str[100];
  int i;

  int width = 640, height = 480;
  int viewCount = 1, objectCount = 1;
  int objectId = 0, viewIdx = 0, objectIdx = 0;

  Timer t;

  //result visualisation
  ImageUChar4* ResultImage = new ImageUChar4(width, height);

  // ---------------------------------------------------------------------------
  //input image
  //camera = 24 bit colour rgb
  ImageUChar4* camera = new ImageUChar4(width, height);
  ImageUtils::Instance()->LoadImageFromFile(camera, (char*)sSrcImage.c_str());

  //objects allocation + initialisation: 3d model in obj required
  Object3D **objects = new Object3D*[objectCount];

  std::cout<<"\n==[APP] Init Model =="<<std::endl;
  objects[objectIdx] = new Object3D(objectId, viewCount, (char*)sModelPath.c_str(), width, height);

  // ---------------------------------------------------------------------------
  //views allocation + initialisation: camera calibration (artoolkit format) required
  std::cout<<"\n==[APP] Init CameraMatrix =="<<std::endl;
  View3D **views = new View3D*[viewCount];
  views[viewIdx] = new View3D(0, (char*)sCameraMatrix.c_str(), width, height);


  // ---------------------------------------------------------------------------
  //histogram initialisation
  //source = 24 bit colour rgb
  //mask = 24 bit black/white png - white represents object
  //videoMask = 24 bit black/white png - white represents parts of the image that are usable
  std::cout<<"\n==[APP] Init Target ROI =="<<std::endl;
  ImageUtils::Instance()->LoadImageFromFile(views[viewIdx]->videoMask,
                                            (char*)sTargetMask.c_str());

  ImageUtils::Instance()->LoadImageFromFile(objects[objectIdx]->histSources[viewIdx],
                                            (char*)sHistSrc.c_str());

  ImageUtils::Instance()->LoadImageFromFile(objects[objectIdx]->histMasks[viewIdx],
                                            (char*)sHistMask.c_str(), objectIdx+1);

  HistogramEngine::Instance()->UpdateVarBinHistogram(
        objects[objectIdx], views[viewIdx], objects[objectIdx]->histSources[viewIdx],
        objects[objectIdx]->histMasks[viewIdx], views[viewIdx]->videoMask);


  // ---------------------------------------------------------------------------
  //iteration configuration for one object
  IterationConfiguration *iterConfig = new IterationConfiguration();
  iterConfig->width = width; iterConfig->height = height;
  iterConfig->iterViewIds[viewIdx] = 0;
  iterConfig->iterObjectCount[viewIdx] = 1;
  iterConfig->levelSetBandSize = 8;
  iterConfig->iterObjectIds[viewIdx][objectIdx] = 0;
  iterConfig->iterViewCount = 1;
  iterConfig->iterCount = 3;

  //step size per object and view
  objects[objectIdx]->stepSize[viewIdx] = new StepSize3D(0.05f, 0.1f, 0.1f, 0.1f);

  //initial pose per object and view
  // Notice the input pose here is angle, not radians for the rotation part
  //  objects[objectIdx]->initialPose[viewIdx]->SetFrom(
  //        -1.98f, -2.90f, 37.47f, -40.90f, -207.77f, 27.48f);

  // for blue car demo
  //  objects[objectIdx]->initialPose[viewIdx]->SetFrom( -3.0f,-4.5f,28.f, -220.90f, -207.77f, 87.48f);

  // for red can demo
//  objects[objectIdx]->initialPose[viewIdx]->SetFrom(
//        1.0f, 3.0f, 30.f, 180.f, 80.f, 60.f);
  objects[objectIdx]->initialPose[viewIdx]->SetFrom(
        -1.98f, -2.90f, 37.47f, -40.90f, -207.77f, 27.48f);

  //primary initilisation
  OptimisationEngine::Instance()->Initialise(width, height);

  //register camera image with main engine
  OptimisationEngine::Instance()->RegisterViewImage(views[viewIdx], camera);

  // ---------------------------------------------------------------------------
  std::cout<<"\n==[APP] Rendering object initial pose.. =="<<std::endl;
  VisualisationEngine::Instance()->GetImage(
        ResultImage, GETIMAGE_PROXIMITY,
        objects[objectIdx], views[viewIdx],
        objects[objectIdx]->initialPose[viewIdx]);

  cv::Mat ResultMat(height,width,CV_8UC4, ResultImage->pixels);
  cv::imshow("initial pose", ResultMat);
  cv::waitKey(1000);

  std::cout<<"[App] Finish Rendered object initial pose."<<std::endl;

  for (i=3; i<4; i++)
  {
    switch (i)
    {
    case 0:
      iterConfig->useCUDAEF = true;
      iterConfig->useCUDARender = true;
      break;
    case 1:
      iterConfig->useCUDAEF = false;
      iterConfig->useCUDARender = true;
      break;
    case 2:
      iterConfig->useCUDAEF = true;
      iterConfig->useCUDARender = false;
      break;
    case 3:
      iterConfig->useCUDAEF = false;
      iterConfig->useCUDARender = false;
      break;
    }

    printf("======= mode: useCUDAAEF: %d, use CUDARender %d ========;\n",
           iterConfig->useCUDAEF, iterConfig->useCUDARender);

    sprintf(str, "/home/rlvc/Workspace/0_code/PWP3D/Files/Results/result%04d.png", i);

    //main processing
    t.restart();
    OptimisationEngine::Instance()->Minimise(objects, views, iterConfig);
    t.check("Iteration");

    //result plot
    VisualisationEngine::Instance()->GetImage(
          ResultImage, GETIMAGE_PROXIMITY,
          objects[objectIdx], views[viewIdx], objects[objectIdx]->pose[viewIdx]);

    //result save to file
    //    ImageUtils::Instance()->SaveImageToFile(result, str);
    cv::Mat ResultMat(height,width,CV_8UC4, ResultImage->pixels);
    cv::imshow("result", ResultMat);
    cv::waitKey(2000);

    printf("final pose result %f %f %f %f %f %f %f\n\n",
           objects[objectIdx]->pose[viewIdx]->translation->x,
           objects[objectIdx]->pose[viewIdx]->translation->y,
           objects[objectIdx]->pose[viewIdx]->translation->z,
           objects[objectIdx]->pose[viewIdx]->rotation->vector4d.x,
           objects[objectIdx]->pose[viewIdx]->rotation->vector4d.y,
           objects[objectIdx]->pose[viewIdx]->rotation->vector4d.z,
           objects[objectIdx]->pose[viewIdx]->rotation->vector4d.w);
  }

  //posteriors plot
  sprintf(str, "/home/rlvc/Workspace/0_code/PWP3D/Files/Results/posteriors.png");
  VisualisationEngine::Instance()->GetImage(
        ResultImage, GETIMAGE_POSTERIORS,
        objects[objectIdx], views[viewIdx], objects[objectIdx]->pose[viewIdx]);

  ImageUtils::Instance()->SaveImageToFile(ResultImage, str);

  //primary engine destructor
  OptimisationEngine::Instance()->Shutdown();

  for (i = 0; i<objectCount; i++) delete objects[i];
  delete objects;

  for (i = 0; i<viewCount; i++) delete views[i];
  delete views;

  delete ResultImage;

  std::cout<<"Exit pwp3D app successfully."<<std::endl;

  return 0;
}

int main_iter(char* mask_format, char* pose_format, int current_round)
{
    float f_init_pose[6] = {0};
    char cur_mask_str[256];
    char nxt_mask_str[256];
    char cur_pose_str[256];
    char nxt_pose_str[256];
    std::cout << "current round: " << current_round << std::endl;
    sprintf(cur_mask_str, mask_format, current_round);
    std::cout << "current mask path: " << cur_mask_str << std::endl;
    sprintf(cur_pose_str, pose_format, current_round);
    std::cout << "current pose path: " << cur_pose_str << std::endl;
    FILE* f = fopen(cur_pose_str, "r");
    fscanf(f, "%f, %f, %f, %f, %f, %f", &f_init_pose[0], &f_init_pose[1], &f_init_pose[2], &f_init_pose[3], &f_init_pose[4], &f_init_pose[5]);
    fclose(f);
    std::cout << "current pose: " << f_init_pose[0] << " " << f_init_pose[1] << " " << f_init_pose[2] << " " << f_init_pose[3] << " " << f_init_pose[4] << " " << f_init_pose[5] << std::endl;
    int next_round = current_round + 1;
    sprintf(nxt_mask_str, mask_format, next_round);
    std::cout << "next mask path: " << nxt_mask_str << std::endl;
    sprintf(nxt_pose_str, pose_format, next_round);
    std::cout << "next pose path: " << nxt_pose_str << std::endl;

    std::string sModelPath =      "/home/rlvc/Workspace/0_code/PWP3D/Files/Models/Renderer/long.obj";
    std::string sSrcImage =       "/home/rlvc/Workspace/0_code/PWP3D/Files/Images/Red.png";
    std::string sCameraMatrix =   "/home/rlvc/Workspace/0_code/PWP3D/Files/CameraCalibration/900nc.cal";
    std::string sTargetMask =     "/home/rlvc/Workspace/0_code/PWP3D/Files/Masks/Red_MaskT.png";//480p_All_VideoMask
    std::string sHistSrc =        "/home/rlvc/Workspace/0_code/PWP3D/Files/Masks/Red_Source.png";
    std::string sHistMask(cur_mask_str);

    // ---------------------------------------------------------------------------
    char str[100];
    int i;

    int width = 640, height = 480;
    int viewCount = 1, objectCount = 1;
    int objectId = 0, viewIdx = 0, objectIdx = 0;

    Timer t;

    //result visualisation
    ImageUChar4* ResultImage = new ImageUChar4(width, height);

    // ---------------------------------------------------------------------------
    //input image
    //camera = 24 bit colour rgb
    ImageUChar4* camera = new ImageUChar4(width, height);
    ImageUtils::Instance()->LoadImageFromFile(camera, (char*)sSrcImage.c_str());

    //objects allocation + initialisation: 3d model in obj required
    Object3D **objects = new Object3D*[objectCount];

//  std::cout<<"\n==[APP] Init Model =="<<std::endl;
    objects[objectIdx] = new Object3D(objectId, viewCount, (char*)sModelPath.c_str(), width, height);

    // ---------------------------------------------------------------------------
    //views allocation + initialisation: camera calibration (artoolkit format) required
//  std::cout<<"\n==[APP] Init CameraMatrix =="<<std::endl;
    View3D **views = new View3D*[viewCount];
    views[viewIdx] = new View3D(0, (char*)sCameraMatrix.c_str(), width, height);


    // ---------------------------------------------------------------------------
    //histogram initialisation
    //source = 24 bit colour rgb
    //mask = 24 bit black/white png - white represents object
    //videoMask = 24 bit black/white png - white represents parts of the image that are usable
//  std::cout<<"\n==[APP] Init Target ROI =="<<std::endl;
    ImageUtils::Instance()->LoadImageFromFile(views[viewIdx]->videoMask,
                                              (char*)sTargetMask.c_str());

    ImageUtils::Instance()->LoadImageFromFile(objects[objectIdx]->histSources[viewIdx],
                                              (char*)sHistSrc.c_str());

    ImageUtils::Instance()->LoadImageFromFile(objects[objectIdx]->histMasks[viewIdx],
                                              (char*)sHistMask.c_str(), objectIdx+1);
//      cv::Mat ResultMat00(480,640,CV_8UC1, objects[objectIdx]->histMasks[viewIdx]->pixels);
//      cv::Mat ResultMat0 = ResultMat00.clone();
//  for (int k = 0; k < 480; ++k) {
//    for (int j = 0; j < 640; ++j) {
//      ResultMat0.at<uchar>(k,j) *= 255;
//    }
//  }
//      cv::imshow("current mask", ResultMat0);
//      cv::waitKey(1000);
    HistogramEngine::Instance()->UpdateVarBinHistogram(
            objects[objectIdx], views[viewIdx], objects[objectIdx]->histSources[viewIdx],
            objects[objectIdx]->histMasks[viewIdx], views[viewIdx]->videoMask);


    // ---------------------------------------------------------------------------
    //iteration configuration for one object
    IterationConfiguration *iterConfig = new IterationConfiguration();
    iterConfig->width = width; iterConfig->height = height;
    iterConfig->iterViewIds[viewIdx] = 0;
    iterConfig->iterObjectCount[viewIdx] = 1;
    iterConfig->levelSetBandSize = 8;
    iterConfig->iterObjectIds[viewIdx][objectIdx] = 0;
    iterConfig->iterViewCount = 1;
    iterConfig->iterCount = 1;

    //step size per object and view
    objects[objectIdx]->stepSize[viewIdx] = new StepSize3D(0.05f, 0.1f, 0.1f, 0.1f);

    //initial pose per object and view
    // Notice the input pose here is angle, not radians for the rotation part
//    objects[objectIdx]->initialPose[viewIdx]->SetFrom(
//          -1.98f, -2.90f, 37.47f, -40.90f, -207.77f, 27.48f);

    // for blue car demo
    //  objects[objectIdx]->initialPose[viewIdx]->SetFrom( -3.0f,-4.5f,28.f, -220.90f, -207.77f, 87.48f);

    // for red can demo
//  objects[objectIdx]->initialPose[viewIdx]->SetFrom(
//        1.0f, 3.0f, 30.f, 180.f, 80.f, 60.f);
    objects[objectIdx]->initialPose[viewIdx]->SetFrom(
            f_init_pose[0], f_init_pose[1],f_init_pose[2],f_init_pose[3],f_init_pose[4],f_init_pose[5]);
//          -1.98f, -2.90f, 37.47f, -40.90f, -207.77f, 27.48f);
//          -1.98f, -2.90f, 37.47f, -40.90f, -207.77f, 27.48f
//          step_ini_pose[0], step_ini_pose[1],step_ini_pose[2],step_ini_pose[3],step_ini_pose[4],step_ini_pose[5]
//          -2.666169, -2.808708, 37.354477, -29.214140, 149.808548, 26.329084
//          -2.740106, -2.723469, 37.318462, -27.502556, 149.059219, 24.947006
//          -2.787121, -2.647153, 37.272305, -25.237589, 147.889771, 23.575155
//          -2.825250, -2.590459, 37.214016, -23.274599, 147.365265, 21.962433
//          -2.853781, -2.525093, 37.151295, -21.680292, 147.083649, 20.411165
//          -2.865823, -2.466633, 37.084679, -20.059132, 146.805588, 18.893599
//          -2.999189, -2.813742, 37.111271, -41.357132, 151.625153, 28.037361
//  );
//  objects[objectIdx]->initialPose[viewIdx]->SetFrom(5.98f, -14.90f, 26.47f, -30.90f, -157.77f, 337.48f);

    //primary initilisation
    OptimisationEngine::Instance()->Initialise(width, height);

    //register camera image with main engine
    OptimisationEngine::Instance()->RegisterViewImage(views[viewIdx], camera);

    // ---------------------------------------------------------------------------
//  std::cout<<"\n==[APP] Rendering object initial pose.. =="<<std::endl;
    VisualisationEngine::Instance()->GetImage(
            ResultImage, GETIMAGE_PROXIMITY,
            objects[objectIdx], views[viewIdx],
            objects[objectIdx]->initialPose[viewIdx]);

//  cv::Mat ResultMat(height,width,CV_8UC4, ResultImage->pixels);
//  cv::imshow("initial pose", ResultMat);
//  cv::waitKey(1000);

//  std::cout<<"[App] Finish Rendered object initial pose."<<std::endl;

    for (i=3; i<4; i++)
    {
        switch (i)
        {
            case 0:
                iterConfig->useCUDAEF = true;
                iterConfig->useCUDARender = true;
                break;
            case 1:
                iterConfig->useCUDAEF = false;
                iterConfig->useCUDARender = true;
                break;
            case 2:
                iterConfig->useCUDAEF = true;
                iterConfig->useCUDARender = false;
                break;
            case 3:
                iterConfig->useCUDAEF = false;
                iterConfig->useCUDARender = false;
                break;
        }

//    printf("======= mode: useCUDAAEF: %d, use CUDARender %d ========;\n",
//           iterConfig->useCUDAEF, iterConfig->useCUDARender);

        sprintf(str, "/home/rlvc/Workspace/0_code/PWP3D/Files/Results/result%04d.png", current_round);

        //main processing
        t.restart();
        OptimisationEngine::Instance()->Minimise(objects, views, iterConfig);
        t.check("Iteration");

        //result plot
        VisualisationEngine::Instance()->GetImage(
                ResultImage, GETIMAGE_PROXIMITY,
                objects[objectIdx], views[viewIdx], objects[objectIdx]->pose[viewIdx]);

        //result save to file
        //    ImageUtils::Instance()->SaveImageToFile(result, str);
        cv::Mat ResultMat(height,width,CV_8UC4, ResultImage->pixels);
        cv::imshow("result", ResultMat);
        cv::waitKey(1000);
        sprintf(str, "/home/rlvc/Workspace/0_code/PWP3D/Files/Masks/RES/result%04d.png", current_round);
        cv::imwrite(str, ResultMat);

//    printf("final pose result %f %f %f %f %f %f %f\n\n",
//           objects[objectIdx]->pose[viewIdx]->translation->x,
//           objects[objectIdx]->pose[viewIdx]->translation->y,
//           objects[objectIdx]->pose[viewIdx]->translation->z,
//           objects[objectIdx]->pose[viewIdx]->rotation->vector4d.x,
//           objects[objectIdx]->pose[viewIdx]->rotation->vector4d.y,
//           objects[objectIdx]->pose[viewIdx]->rotation->vector4d.z,
//           objects[objectIdx]->pose[viewIdx]->rotation->vector4d.w);
        VECTOR3DA euler_rotation;
        objects[objectIdx]->pose[viewIdx]->rotation->ToEuler(&euler_rotation);
//    printf("final rotation %f, %f, %f, %f, %f, %f\n\n",
//           objects[objectIdx]->pose[viewIdx]->translation->x,
//           objects[objectIdx]->pose[viewIdx]->translation->y,
//           objects[objectIdx]->pose[viewIdx]->translation->z,
//           euler_rotation.x, euler_rotation.y, euler_rotation.z);
        f_init_pose[0] = objects[objectIdx]->pose[viewIdx]->translation->x;
        f_init_pose[1] = objects[objectIdx]->pose[viewIdx]->translation->y;
        f_init_pose[2] = objects[objectIdx]->pose[viewIdx]->translation->z;
        f_init_pose[3] = euler_rotation.x;
        f_init_pose[4] = euler_rotation.y;
        f_init_pose[5] = euler_rotation.z;
        FILE* f = fopen(nxt_pose_str, "w");
        fprintf(f, "%f, %f, %f, %f, %f, %f", f_init_pose[0], f_init_pose[1], f_init_pose[2], f_init_pose[3], f_init_pose[4], f_init_pose[5]);
        fclose(f);
    }

//  cv::Mat mask = obmesh.GenMask(objects[objectIdx]->pose[viewIdx]);
//  cv::imshow("mask", mask);
//  cv::waitKey(0);
    //posteriors plot
    sprintf(str, "/home/rlvc/Workspace/0_code/PWP3D/Files/Results/posteriors.png");
//  ResultImage->Clear();
    VisualisationEngine::Instance()->GetImage(
            ResultImage, GETIMAGE_WIREFRAME,
            objects[objectIdx], views[viewIdx], objects[objectIdx]->pose[viewIdx]);

    ImageUtils::Instance()->SaveImageToFile(ResultImage, str);
    cv::Mat mask = cv::imread(str);
//  cv::imshow("WIREFRAME mask", mask);
//  cv::waitKey(1000);
    int row_begin[480];
    int row_end[480];
    for (int i = 0; i < 480; ++i) {
        row_begin[i] = -1;
        row_end[i] = -1;
    }
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            if(mask.at<cv::Vec3b>(i,j)[0] != 0){
                row_begin[i] = j;
                break;
            }
        }
        for (int j = width - 1; j >= 0; --j) {
            if(mask.at<cv::Vec3b>(i,j)[0] != 0){
                row_end[i] = j;
                break;
            }
        }
    }
    for (int i = 0; i < height; ++i) {
        if(row_begin[i] == -1 || row_end[i] == -1 || row_begin[i] > row_end[i])
            continue;
        for (int j = row_begin[i]; j <= row_end[i]; ++j) {
            mask.at<cv::Vec3b>(i,j)[0] = 255;
            mask.at<cv::Vec3b>(i,j)[1] = 255;
            mask.at<cv::Vec3b>(i,j)[2] = 255;
        }
    }
//  cv::imshow("mask", mask);
//  cv::waitKey(1000);
//  sprintf(str, "/home/rlvc/Workspace/0_code/PWP3D/Files/Masks/temp_mask%04d.png", issndex);
    cv::imwrite(nxt_mask_str, mask);

    //primary engine destructor
    OptimisationEngine::Instance()->Shutdown();
//  VisualisationEngine::Instance()->Shutdown();

    for (i = 0; i<objectCount; i++) delete objects[i];
    delete objects;

    for (i = 0; i<viewCount; i++) delete views[i];
    delete views;

    delete ResultImage;

    std::cout<<"Exit pwp3D app successfully."<<std::endl;

    return 0;
}