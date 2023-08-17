// #include "linefeature_tracker.h"
// #include <opencv2/line_descriptor.hpp>
// #include <opencv2/core/utility.hpp>
// #include <opencv2/imgproc.hpp>
// #include <opencv2/features2d.hpp>
// #include <opencv2/highgui.hpp>
// #include "opencv2/ximgproc.hpp"
// #include "opencv2/imgcodecs.hpp"

// LineFeatureTracker::LineFeatureTracker()
// {
//     allfeature_cnt = 0;
//     frame_cnt = 0;
//     sum_time = 0.0;
// }

// void LineFeatureTracker::readIntrinsicParameter(const string &calib_file)
// {
//     ROS_INFO("reading paramerter of camera %s", calib_file.c_str());

//     m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);

//     K_ = m_camera->initUndistortRectifyMap(undist_map1_, undist_map2_);
// }

// vector<Line> LineFeatureTracker::undistortedLineEndPoints()
// {
//     vector<Line> un_lines;
//     un_lines = curframe_->vecLine;

//     ROS_INFO("----IN UNDISTORT LINE END POINTS----");
//     ROS_INFO("un_lines[0].StartPt.x", un_lines[0].StartPt.x);
    
//     float fx = K_.at<float>(0, 0);
//     float fy = K_.at<float>(1, 1);
//     float cx = K_.at<float>(0, 2);
//     float cy = K_.at<float>(1, 2);
//     for (unsigned int i = 0; i < curframe_->vecLine.size(); i++)
//     {
//         un_lines[i].StartPt.x = (curframe_->vecLine[i].StartPt.x - cx) / fx;
//         un_lines[i].StartPt.y = (curframe_->vecLine[i].StartPt.y - cy) / fy;
//         un_lines[i].EndPt.x = (curframe_->vecLine[i].EndPt.x - cx) / fx;
//         un_lines[i].EndPt.y = (curframe_->vecLine[i].EndPt.y - cy) / fy;
//     }
//     return un_lines;
// }

// void LineFeatureTracker::NearbyLineTracking(const vector<Line> forw_lines, const vector<Line> cur_lines,
//                                             vector<pair<int, int>> &lineMatches)
// {

//     float th = 3.1415926 / 9;
//     float dth = 30 * 30;
//     for (size_t i = 0; i < forw_lines.size(); ++i)
//     {
//         Line lf = forw_lines.at(i);
//         Line best_match;
//         size_t best_j = 100000;
//         size_t best_i = 100000;
//         float grad_err_min_j = 100000;
//         float grad_err_min_i = 100000;
//         vector<Line> candidate;

//         // 从 forw --> cur 查找
//         for (size_t j = 0; j < cur_lines.size(); ++j)
//         {
//             Line lc = cur_lines.at(j);
//             // condition 1
//             Point2f d = lf.Center - lc.Center;
//             float dist = d.dot(d);
//             if (dist > dth)
//                 continue; //
//             // condition 2
//             float delta_theta1 = fabs(lf.theta - lc.theta);
//             float delta_theta2 = 3.1415926 - delta_theta1;
//             if (delta_theta1 < th || delta_theta2 < th)
//             {
//                 // std::cout << "theta: "<< lf.theta * 180 / 3.14259 <<" "<< lc.theta * 180 / 3.14259<<" "<<delta_theta1<<" "<<delta_theta2<<std::endl;
//                 candidate.push_back(lc);
//                 // float cost = fabs(lf.image_dx - lc.image_dx) + fabs( lf.image_dy - lc.image_dy) + 0.1 * dist;
//                 float cost = fabs(lf.line_grad_avg - lc.line_grad_avg) + dist / 10.0;

//                 // std::cout<< "line match cost: "<< cost <<" "<< cost - sqrt( dist )<<" "<< sqrt( dist ) <<"\n\n";
//                 if (cost < grad_err_min_j)
//                 {
//                     best_match = lc;
//                     grad_err_min_j = cost;
//                     best_j = j;
//                 }
//             }
//         }
//         if (grad_err_min_j > 50)
//             continue; // 没找到

//         // std::cout<< "!!!!!!!!! minimal cost: "<<grad_err_min_j <<"\n\n";

//         // 如果 forw --> cur 找到了 best, 那我们反过来再验证下
//         if (best_j < cur_lines.size())
//         {
//             // 反过来，从 cur --> forw 查找
//             Line lc = cur_lines.at(best_j);
//             for (int k = 0; k < forw_lines.size(); ++k)
//             {
//                 Line lk = forw_lines.at(k);

//                 // condition 1
//                 Point2f d = lk.Center - lc.Center;
//                 float dist = d.dot(d);
//                 if (dist > dth)
//                     continue; //
//                 // condition 2
//                 float delta_theta1 = fabs(lk.theta - lc.theta);
//                 float delta_theta2 = 3.1415926 - delta_theta1;
//                 if (delta_theta1 < th || delta_theta2 < th)
//                 {
//                     // std::cout << "theta: "<< lf.theta * 180 / 3.14259 <<" "<< lc.theta * 180 / 3.14259<<" "<<delta_theta1<<" "<<delta_theta2<<std::endl;
//                     // candidate.push_back(lk);
//                     // float cost = fabs(lk.image_dx - lc.image_dx) + fabs( lk.image_dy - lc.image_dy) + dist;
//                     float cost = fabs(lk.line_grad_avg - lc.line_grad_avg) + dist / 10.0;

//                     if (cost < grad_err_min_i)
//                     {
//                         grad_err_min_i = cost;
//                         best_i = k;
//                     }
//                 }
//             }
//         }

//         if (grad_err_min_i < 50 && best_i == i)
//         {

//             // std::cout<< "line match cost: "<<grad_err_min_j<<" "<<grad_err_min_i <<"\n\n";
//             lineMatches.push_back(make_pair(best_j, i));
//         }
//         /*
//         vector<Line> l;
//         l.push_back(lf);
//         vector<Line> best;
//         best.push_back(best_match);
//         visualizeLineTrackCandidate(l,forwframe_->img,"forwframe_");
//         visualizeLineTrackCandidate(best,curframe_->img,"curframe_best");
//         visualizeLineTrackCandidate(candidate,curframe_->img,"curframe_");
//         cv::waitKey(0);
//         */
//     }
// }

// // #define NLT
// #ifdef NLT
// void LineFeatureTracker::readImage(const cv::Mat &_img)
// {
//     cv::Mat img;
//     TicToc t_p;
//     frame_cnt++;
//     cv::remap(_img, img, undist_map1_, undist_map2_, CV_INTER_LINEAR);
//     // ROS_INFO("undistortImage costs: %fms", t_p.toc());
//     if (EQUALIZE) // 直方图均衡化
//     {
//         cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
//         clahe->apply(img, img);
//     }

//     bool first_img = false;
//     if (forwframe_ == nullptr) // 系统初始化的第一帧图像
//     {
//         forwframe_.reset(new FrameLines);
//         curframe_.reset(new FrameLines);
//         forwframe_->img = img;
//         curframe_->img = img;
//         first_img = true;
//     }
//     else
//     {
//         forwframe_.reset(new FrameLines); // 初始化一个新的帧
//         forwframe_->img = img;
//     }

//     // step 1: line extraction
//     TicToc t_li;
//     int lineMethod = 2;
//     bool isROI = false;
//     lineDetector ld(lineMethod, isROI, 0, (float)img.cols, 0, (float)img.rows);
//     // ROS_INFO("ld inition costs: %fms", t_li.toc());
//     TicToc t_ld;
//     forwframe_->vecLine = ld.detect(img);

//     for (size_t i = 0; i < forwframe_->vecLine.size(); ++i)
//     {
//         if (first_img)
//             forwframe_->lineID.push_back(allfeature_cnt++);
//         else
//             forwframe_->lineID.push_back(-1); // give a negative id
//     }
//     ROS_INFO("line detect costs: %fms", t_ld.toc());

//     // step 3: junction & line matching
//     if (curframe_->vecLine.size() > 0)
//     {
//         TicToc t_nlt;
//         vector<pair<int, int>> linetracker;
//         NearbyLineTracking(forwframe_->vecLine, curframe_->vecLine, linetracker);
//         ROS_INFO("line match costs: %fms", t_nlt.toc());

//         // 对新图像上的line赋予id值
//         for (int j = 0; j < linetracker.size(); j++)
//         {
//             forwframe_->lineID[linetracker[j].second] = curframe_->lineID[linetracker[j].first];
//         }

//         // show NLT match
//         visualizeLineMatch(curframe_->vecLine, forwframe_->vecLine, linetracker,
//                            curframe_->img, forwframe_->img, "NLT Line Matches", 10, true,
//                            "frame");
//         visualizeLinewithID(forwframe_->vecLine, forwframe_->lineID, forwframe_->img, "forwframe_");
//         visualizeLinewithID(curframe_->vecLine, curframe_->lineID, curframe_->img, "curframe_");
//         stringstream ss;
//         ss << "/home/hyj/datasets/line/" << frame_cnt << ".jpg";
//         // SaveFrameLinewithID(forwframe_->vecLine,forwframe_->lineID,forwframe_->img,ss.str().c_str());
//         waitKey(5);

//         vector<Line> vecLine_tracked, vecLine_new;
//         vector<int> lineID_tracked, lineID_new;
//         // 将跟踪的线和没跟踪上的线进行区分
//         for (size_t i = 0; i < forwframe_->vecLine.size(); ++i)
//         {
//             if (forwframe_->lineID[i] == -1)
//             {
//                 forwframe_->lineID[i] = allfeature_cnt++;
//                 vecLine_new.push_back(forwframe_->vecLine[i]);
//                 lineID_new.push_back(forwframe_->lineID[i]);
//             }
//             else
//             {
//                 vecLine_tracked.push_back(forwframe_->vecLine[i]);
//                 lineID_tracked.push_back(forwframe_->lineID[i]);
//             }
//         }
//         int diff_n = 30 - vecLine_tracked.size(); // 跟踪的线特征少于50了，那就补充新的线特征, 还差多少条线
//         if (diff_n > 0)                           // 补充线条
//         {
//             for (int k = 0; k < vecLine_new.size(); ++k)
//             {
//                 vecLine_tracked.push_back(vecLine_new[k]);
//                 lineID_tracked.push_back(lineID_new[k]);
//             }
//         }

//         forwframe_->vecLine = vecLine_tracked;
//         forwframe_->lineID = lineID_tracked;
//     }
//     curframe_ = forwframe_;
// }
// #endif

// #define MATCHES_DIST_THRESHOLD 30
// void visualize_line_match(Mat imageMat1, Mat imageMat2,
//                           std::vector<KeyLine> octave0_1, std::vector<KeyLine> octave0_2,
//                           std::vector<DMatch> good_matches)
// {
//     //	Mat img_1;
//     cv::Mat img1, img2;
//     if (imageMat1.channels() != 3)
//     {
//         cv::cvtColor(imageMat1, img1, cv::COLOR_GRAY2BGR);
//     }
//     else
//     {
//         img1 = imageMat1;
//     }

//     if (imageMat2.channels() != 3)
//     {
//         cv::cvtColor(imageMat2, img2, cv::COLOR_GRAY2BGR);
//     }
//     else
//     {
//         img2 = imageMat2;
//     }

//     //    srand(time(NULL));
//     int lowest = 0, highest = 255;
//     int range = (highest - lowest) + 1;
//     for (int k = 0; k < good_matches.size(); ++k)
//     {
//         DMatch mt = good_matches[k];

//         KeyLine line1 = octave0_1[mt.queryIdx]; // trainIdx
//         KeyLine line2 = octave0_2[mt.trainIdx]; // queryIdx

//         unsigned int r = lowest + int(rand() % range);
//         unsigned int g = lowest + int(rand() % range);
//         unsigned int b = lowest + int(rand() % range);
//         cv::Point startPoint = cv::Point(int(line1.startPointX), int(line1.startPointY));
//         cv::Point endPoint = cv::Point(int(line1.endPointX), int(line1.endPointY));
//         cv::line(img1, startPoint, endPoint, cv::Scalar(r, g, b), 2, 8);

//         cv::Point startPoint2 = cv::Point(int(line2.startPointX), int(line2.startPointY));
//         cv::Point endPoint2 = cv::Point(int(line2.endPointX), int(line2.endPointY));
//         cv::line(img2, startPoint2, endPoint2, cv::Scalar(r, g, b), 2, 8);
//         cv::line(img2, startPoint, startPoint2, cv::Scalar(0, 0, 255), 1, 8);
//         cv::line(img2, endPoint, endPoint2, cv::Scalar(0, 0, 255), 1, 8);
//     }
//     /* plot matches */
//     /*
//     cv::Mat lsd_outImg;
//     std::vector<char> lsd_mask( lsd_matches.size(), 1 );
//     drawLineMatches( imageMat1, octave0_1, imageMat2, octave0_2, good_matches, lsd_outImg, Scalar::all( -1 ), Scalar::all( -1 ), lsd_mask,
//     DrawLinesMatchesFlags::DEFAULT );

//     imshow( "LSD matches", lsd_outImg );
//     */
//     imshow("LSD matches1", img1);
//     imshow("LSD matches2", img2);
//     waitKey(1);
// }

// void visualize_line_match(Mat imageMat1, Mat imageMat2,
//                           std::vector<KeyLine> octave0_1, std::vector<KeyLine> octave0_2,
//                           std::vector<bool> good_matches)
// {
//     //	Mat img_1;
//     cv::Mat img1, img2;
//     if (imageMat1.channels() != 3)
//     {
//         cv::cvtColor(imageMat1, img1, cv::COLOR_GRAY2BGR);
//     }
//     else
//     {
//         img1 = imageMat1;
//     }

//     if (imageMat2.channels() != 3)
//     {
//         cv::cvtColor(imageMat2, img2, cv::COLOR_GRAY2BGR);
//     }
//     else
//     {
//         img2 = imageMat2;
//     }

//     //    srand(time(NULL));
//     int lowest = 0, highest = 255;
//     int range = (highest - lowest) + 1;
//     for (int k = 0; k < good_matches.size(); ++k)
//     {

//         if (!good_matches[k])
//             continue;

//         KeyLine line1 = octave0_1[k]; // trainIdx
//         KeyLine line2 = octave0_2[k]; // queryIdx

//         unsigned int r = lowest + int(rand() % range);
//         unsigned int g = lowest + int(rand() % range);
//         unsigned int b = lowest + int(rand() % range);
//         cv::Point startPoint = cv::Point(int(line1.startPointX), int(line1.startPointY));
//         cv::Point endPoint = cv::Point(int(line1.endPointX), int(line1.endPointY));
//         cv::line(img1, startPoint, endPoint, cv::Scalar(r, g, b), 2, 8);

//         cv::Point startPoint2 = cv::Point(int(line2.startPointX), int(line2.startPointY));
//         cv::Point endPoint2 = cv::Point(int(line2.endPointX), int(line2.endPointY));
//         cv::line(img2, startPoint2, endPoint2, cv::Scalar(r, g, b), 2, 8);
//         cv::line(img2, startPoint, startPoint2, cv::Scalar(0, 0, 255), 1, 8);
//         cv::line(img2, endPoint, endPoint2, cv::Scalar(0, 0, 255), 1, 8);
//     }
//     /* plot matches */
//     /*
//     cv::Mat lsd_outImg;
//     std::vector<char> lsd_mask( lsd_matches.size(), 1 );
//     drawLineMatches( imageMat1, octave0_1, imageMat2, octave0_2, good_matches, lsd_outImg, Scalar::all( -1 ), Scalar::all( -1 ), lsd_mask,
//     DrawLinesMatchesFlags::DEFAULT );

//     imshow( "LSD matches", lsd_outImg );
//     */
//     imshow("LSD matches1", img1);
//     imshow("LSD matches2", img2);
//     waitKey(1);
// }

// void LineFeatureTracker::readImage(const cv::Mat &_img)
// {

//     // bool first_img = false;
//     // if (forwframe_ == nullptr) // The first frame image of system initialization
//     // {
//     //     forwframe_.reset(new FrameLines);
//     //     curframe_.reset(new FrameLines);
//     //     forwframe_->img = img;
//     //     curframe_->img = img;
//     //     first_img = true;
//     // }

//     // else
//     // {
//     //     forwframe_.reset(new FrameLines); // initialize a new frame
//     //     forwframe_->img = img;
//     // }
    
//     // m_forwardedLines.clear();
//     // m_newLines.clear();

//     // if (m_prevLines.size() > 0)
//     // {
//     //     // stack end points from lines detected in previous frame
//     //     // and track these points using optical flow in current frame
//     //     std::vector<cv::Point2f> prevPoints, forwardedPoints, prevPointsReversed;
//     //     for (const auto& l : m_prevLines)
//     //     {
//     //         prevPoints.emplace_back(l.startPixelCoord);
//     //         prevPoints.emplace_back(l.endPixelCoord);
//     //     }

//     //     // get forwarded points and reverse previous points
//     //     // using bi-directional optical flow based KLT tracker
//     //     cv::Size windowSize(752, 480);
//     //     std::vector<uchar> status, statusReverse;
//     //     std::vector<float> err, errReverse;
//     //     cv::calcOpticalFlowPyrLK(curframe_->img,
//     //                              forwframe_->img,
//     //                              prevPoints,
//     //                              forwardedPoints,
//     //                              status,
//     //                              err,
//     //                              windowSize,
//     //                              3);
//     //     cv::calcOpticalFlowPyrLK(forwframe_->img,
//     //                              curframe_->img,
//     //                              forwardedPoints,
//     //                              prevPointsReversed,
//     //                              statusReverse,
//     //                              errReverse,
//     //                              windowSize,
//     //                              3);

//     //     // // get forwarded lines from forwarded points
//     //     // for (std::size_t idx = 0; idx < forwardedPoints.size(); idx += 2)
//     //     // {
//     //     //     Line2D l;
//     //     //     l.startPixelCoord = forwardedPoints[idx];
//     //     //     l.endPixelCoord = forwardedPoints[idx + 1];

//     //     //     camera->unprojectPoint(l.startPixelCoord, l.startPoint);
//     //     //     camera->unprojectPoint(l.endPixelCoord, l.endPoint);
//     //     //     m_forwardedLines.emplace_back(l);
//     //     // }

//     //     // // if reverse optical flow from forwarded points
//     //     // // do not match the previous points, then mark the point as untracked
//     //     // for (std::size_t idx = 0; idx < prevPoints.size(); idx++)
//     //     // {
//     //     //     const auto& p1 = prevPoints[idx];
//     //     //     const auto& p2 = prevPointsReversed[idx];
//     //     //     if (std::abs(p1.x - p2.x) > m_trackingPixelErrThreshold
//     //     //         || std::abs(p1.y - p2.y) > m_trackingPixelErrThreshold)
//     //     //     {
//     //     //         status[idx] = static_cast<uchar>(0);
//     //     //     }
//     //     // }

//     //     // // set tracked features failing border check as untracked features
//     //     // for (std::size_t idx = 0; idx < forwardedPoints.size(); idx++)
//     //     // {
//     //     //     const auto& pt = forwardedPoints[idx];
//     //     //     if (static_cast<int>(status[idx]) && !camera->inBorder(pt, m_borderSize))
//     //     //     {
//     //     //         status[idx] = static_cast<uchar>(0);
//     //     //     }
//     //     // }

//     //     // reduceLinesVector(status, m_prevLines);
//     //     // reduceLinesVector(status, m_forwardedLines);
//     //     // reduceLinesVector(status, m_trackedIDs);
//     //     // reduceLinesVector(status, m_trackCount);
//     // }

//     // // increment the track count of persisting features
//     // for (auto& track : linetrack_cnt)
//     // {
//     //     track++;
//     // }

//     // // // set mask for image sub space with tracked features
//     // // setMask(camera);

//     // // // detect and add new lines
//     // // cv::Mat maskedImg;
//     // // currImg.copyTo(maskedImg, m_mask);

//     // m_fld->detect(maskedImg, m_detectedLines);
//     // for (auto& dLine : m_detectedLines)
//     // {
//     //     Line2D l;
//     //     l.startPixelCoord.x = dLine(0);
//     //     l.startPixelCoord.y = dLine(1);
//     //     l.endPixelCoord.x = dLine(2);
//     //     l.endPixelCoord.y = dLine(3);

//     //     // camera->unprojectPoint(l.startPixelCoord, l.startPoint);
//     //     // camera->unprojectPoint(l.endPixelCoord, l.endPoint);
//     //     // m_newLines.emplace_back(l);
//     // }

//     // // for (auto& l : m_newLines)
//     // // {
//     // //     m_forwardedLines.emplace_back(l);
//     // //     m_trackedIDs.emplace_back(m_featureID++);
//     // //     m_trackCount.emplace_back(1);
//     // // }

//     // m_prevLines = m_forwardedLines;
//     // trackedLines.lines = m_forwardedLines;
//     // linetrack_cnt = m_trackCount;
//     // //ids = m_trackedIDs;





//     cv::Mat img;
//     TicToc t_p;
//     frame_cnt++;

//     ROS_INFO("IN readImage (1)");

//     cv::remap(_img, img, undist_map1_, undist_map2_, cv::INTER_LINEAR);
//     ROS_INFO("IN readImage (2)");

//     //    cv::imshow("lineimg",img);
//     //    cv::waitKey(1);
//     // ROS_INFO("undistortImage costs: %fms", t_p.toc());
//     if (EQUALIZE) // 直方图均衡化
//     {
//         cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
//         clahe->apply(img, img);
//     }

//     ROS_INFO("IN readImage (3)");

//     bool first_img = false;
//     if (forwframe_ == nullptr) // The first frame image of system initialization
//     {
//         forwframe_.reset(new FrameLines);
//         curframe_.reset(new FrameLines);
//         forwframe_->img = img;
//         curframe_->img = img;
//         first_img = true;
//     }

//     else
//     {
//         forwframe_.reset(new FrameLines); // initialize a new frame
//         forwframe_->img = img;
//     }

//     ROS_INFO("IN readImage (4)");

//     // step 1: line extraction
//     TicToc t_li;

//     std::vector<KeyLine> lsd, keylsd;
//     // std::vector<KeyLine> keylsd;

//     int length_threshold = 10;
//     float distance_threshold = 1.41421356f;
//     double canny_th1 = 50.0;
//     double canny_th2 = 50.0;
//     int canny_aperture_size = 3;
//     bool do_merge = false;
//     Ptr<cv::ximgproc::FastLineDetector> fld = cv::ximgproc::createFastLineDetector(length_threshold,
//                                                        distance_threshold, canny_th1, canny_th2, canny_aperture_size, do_merge);
//     ROS_INFO("IN readImage (4.1)");

//     vector<Vec4f> lines;

//     vector<cv::Point2f> points, nextpoints;

//     cv::line_descriptor::KeyLine templsd;
//     cv::Point2f tempPoint;


//     fld->detect(img, lines);

//     ROS_INFO("IN readImage (4.2)");

//     for ( size_t i = 0; i < lines.size(); i++ )
//     {

//         templsd.startPointX = lines[i][0]; //x1
//         templsd.startPointY = lines[i][1]; //y1
//         templsd.endPointX = lines[i][2]; //x2
//         templsd.endPointY = lines[i][3]; //y2

//         tempPoint.x = lines[i][0];
//         tempPoint.y = lines[i][1];
         
//         points.emplace_back(tempPoint);

//         tempPoint.x = lines[i][2];
//         tempPoint.y = lines[i][3];
        
//         points.emplace_back(tempPoint);


//         lsd.push_back(templsd);
//         //lsd[i].startPointX = lines[i][0];
        

//     }    

//     std::vector<uchar> status, statusReverse;
//     std::vector<float> err, errReverse;
//     cv::Size windowSize(752, 480);

//     cv::calcOpticalFlowPyrLK(curframe_->img,
//                                  forwframe_->img,
//                                  points,
//                                  nextpoints,
//                                  status,
//                                  err,
//                                  windowSize,
//                                  3);

//     sum_time += t_li.toc();

//     ROS_INFO("line detect costs: %fms", t_li.toc());

//     ROS_INFO("nextpoints[0].x: %f", nextpoints[0].x);

//     Mat lbd_descr, keylbd_descr;
//     TicToc t_lbd;


//     //////////////////////////
//     // for (int i = 0; i < (int)lsd.size(); i++)
//     // {
//     //     if (lsd[i].octave == 0 && lsd[i].lineLength >= 30)
//     //     {
//     //         keylsd.push_back(lsd[i]);
//     //         keylbd_descr.push_back(lbd_descr.row(i));
//     //     }
//     // }


//     // //    ROS_INFO("lbd_descr detect costs: %fms", keylsd.size() * t_lbd.toc() / lsd.size() );
//     // sum_time += keylsd.size() * t_lbd.toc() / lsd.size();
//     // ///////////////

//     // // forwframe_->keylsd = keylsd;
//     // // forwframe_->lbd_descr = keylbd_descr;

//     // for (size_t i = 0; i < nextpoints.size(); ++i)
//     // {
//     //     if (first_img)
//     //         forwframe_->lineID.push_back(allfeature_cnt++);
//     //     else
//     //         forwframe_->lineID.push_back(-1); // give a negative id
//     // }

//     // ROS_INFO("IN readImage (9)");

//     if (points.size() > 0)
//     {

//     //     // /* compute matches */
//     //     // TicToc t_match;
//     //     // std::vector<DMatch> lsd_matches;
//     //     // Ptr<BinaryDescriptorMatcher> bdm_;
//     //     // bdm_ = BinaryDescriptorMatcher::createBinaryDescriptorMatcher();
//     //     // bdm_->match(forwframe_->lbd_descr, curframe_->lbd_descr, lsd_matches);
//     //     // //        ROS_INFO("lbd_macht costs: %fms", t_match.toc());
//     //     // sum_time += t_match.toc();
//     //     // mean_time = sum_time / frame_cnt;
//     //     // ROS_INFO("line feature tracker mean costs: %fms", mean_time);

//     //     // /* select best matches */
//     //     // std::vector<DMatch> good_matches;
//     //     // std::vector<KeyLine> good_Keylines;
//     //     // good_matches.clear();
//     //     // for (int i = 0; i < (int)lsd_matches.size(); i++)
//     //     // {
//     //     //     if (lsd_matches[i].distance < MATCHES_DIST_THRESHOLD)
//     //     //     {

//     //     //         DMatch mt = lsd_matches[i];
//     //     //         KeyLine line1 = forwframe_->keylsd[mt.queryIdx];
//     //     //         KeyLine line2 = curframe_->keylsd[mt.trainIdx];
//     //     //         Point2f serr = line1.getStartPoint() - line2.getStartPoint();
//     //     //         Point2f eerr = line1.getEndPoint() - line2.getEndPoint();
//     //     //         if ((serr.dot(serr) < 60 * 60) && (eerr.dot(eerr) < 60 * 60)) // 线段在图像里不会跑得特别远
//     //     //             good_matches.push_back(lsd_matches[i]);
//     //     //     }
//     //     // }

//     //     // std::cout << forwframe_->lineID.size() << " " << curframe_->lineID.size();
//     //     // for (int k = 0; k < good_matches.size(); ++k)
//     //     // {
//     //     //     DMatch mt = good_matches[k];
//     //     //     forwframe_->lineID[mt.queryIdx] = curframe_->lineID[mt.trainIdx];
//     //     // }
//     //     // visualize_line_match(forwframe_->img.clone(), curframe_->img.clone(), forwframe_->keylsd, curframe_->keylsd, good_matches);

//     //     vector<KeyLine> vecLine_tracked, vecLine_new;
//     //     vector<int> lineID_tracked, lineID_new;
//     //     Mat DEscr_tracked, Descr_new;

//     //     // Distinguish between traced lines and untraceable lines
//     //     for (size_t i = 0; i < forwframe_->keylsd.size(); ++i)
//     //     {
//     //         if (forwframe_->lineID[i] == -1)
//     //         {
//     //             forwframe_->lineID[i] = allfeature_cnt++;
//     //             // vecLine_new.push_back(forwframe_->keylsd[i]);
//     //             lineID_new.push_back(forwframe_->lineID[i]);
//     //             // Descr_new.push_back(forwframe_->lbd_descr.row(i));
//     //         }
//     //         else
//     //         {
//     //             // vecLine_tracked.push_back(forwframe_->keylsd[i]);
//     //             lineID_tracked.push_back(forwframe_->lineID[i]);
//     //             // DEscr_tracked.push_back(forwframe_->lbd_descr.row(i));
//     //         }
//     //     }
//     //     int diff_n = 50 - vecLine_tracked.size(); // If the tracked line features are less than 50, then add new line features, how many lines are left
//     //     if (diff_n > 0)                           // Supplementary lines
//     //     {

//     //         for (int k = 0; k < vecLine_new.size(); ++k)
//     //         {
//     //             vecLine_tracked.push_back(vecLine_new[k]);
//     //             lineID_tracked.push_back(lineID_new[k]);
//     //             DEscr_tracked.push_back(Descr_new.row(k));
//     //         }
//     //     }

//         // forwframe_->keylsd = nextpoints;
//         // forwframe_->lineID = lineID_tracked;
//         // forwframe_->lbd_descr = DEscr_tracked;
//     }


//     // Convert opencv's KeyLine data to Jige's Line
//     for (int j = 0; j < nextpoints.size(); ++j)
//     {
//         Line l;
//         KeyLine lsd = forwframe_->keylsd[j];
//         l.StartPt = nextpoints[j];
//         l.EndPt = nextpoints[j+1];
//         l.length = sqrt(pow(nextpoints[j+1].x - nextpoints[j].x, 2) + pow(nextpoints[j+1].y - nextpoints[j].y, 2) * 1.0);
//         forwframe_->vecLine.push_back(l);
//     }
//     curframe_ = forwframe_;
// }
