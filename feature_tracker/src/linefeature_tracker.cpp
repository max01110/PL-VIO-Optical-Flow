#include "linefeature_tracker.h"


LineFeatureTracker::LineFeatureTracker()
{
    allfeature_cnt = 0;
    frame_cnt = 0;
    sum_time = 0.0;
}

void LineFeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());

    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
    K_ = m_camera->initUndistortRectifyMap(undist_map1_,undist_map2_);    

}

vector<Line> LineFeatureTracker::undistortedLineEndPoints()
{
    vector<Line> un_lines;
    un_lines = curframe_->vecLine;
    float fx = K_.at<float>(0, 0);
    float fy = K_.at<float>(1, 1);
    float cx = K_.at<float>(0, 2);
    float cy = K_.at<float>(1, 2);

    for (unsigned int i = 0; i <curframe_->vecLine.size(); i++)
    {
        un_lines[i].StartPt.x = (curframe_->vecLine[i].StartPt.x - cx)/fx;
        un_lines[i].StartPt.y = (curframe_->vecLine[i].StartPt.y - cy)/fy;
        un_lines[i].EndPt.x = (curframe_->vecLine[i].EndPt.x - cx)/fx;
        un_lines[i].EndPt.y = (curframe_->vecLine[i].EndPt.y - cy)/fy;
    }

    return un_lines;
}

void LineFeatureTracker::NearbyLineTracking(const vector<Line> forw_lines, const vector<Line> cur_lines,
                                            vector<pair<int, int> > &lineMatches) {

    float th = 3.1415926/9;
    float dth = 30 * 30;
    for (size_t i = 0; i < forw_lines.size(); ++i) {
        Line lf = forw_lines.at(i);
        Line best_match;
        size_t best_j = 100000;
        size_t best_i = 100000;
        float grad_err_min_j = 100000;
        float grad_err_min_i = 100000;
        vector<Line> candidate;

        // 从 forw --> cur 查找
        for(size_t j = 0; j < cur_lines.size(); ++j) {
            Line lc = cur_lines.at(j);
            // condition 1
            Point2f d = lf.Center - lc.Center;
            float dist = d.dot(d);
            if( dist > dth) continue;  //
            // condition 2
            float delta_theta1 = fabs(lf.theta - lc.theta);
            float delta_theta2 = 3.1415926 - delta_theta1;
            if( delta_theta1 < th || delta_theta2 < th)
            {
                //std::cout << "theta: "<< lf.theta * 180 / 3.14259 <<" "<< lc.theta * 180 / 3.14259<<" "<<delta_theta1<<" "<<delta_theta2<<std::endl;
                candidate.push_back(lc);
                //float cost = fabs(lf.image_dx - lc.image_dx) + fabs( lf.image_dy - lc.image_dy) + 0.1 * dist;
                float cost = fabs(lf.line_grad_avg - lc.line_grad_avg) + dist/10.0;

                //std::cout<< "line match cost: "<< cost <<" "<< cost - sqrt( dist )<<" "<< sqrt( dist ) <<"\n\n";
                if(cost < grad_err_min_j)
                {
                    best_match = lc;
                    grad_err_min_j = cost;
                    best_j = j;
                }
            }

        }
        if(grad_err_min_j > 50) continue;  // 没找到

        //std::cout<< "!!!!!!!!! minimal cost: "<<grad_err_min_j <<"\n\n";

        // 如果 forw --> cur 找到了 best, 那我们反过来再验证下
        if(best_j < cur_lines.size())
        {
            // 反过来，从 cur --> forw 查找
            Line lc = cur_lines.at(best_j);
            for (int k = 0; k < forw_lines.size(); ++k)
            {
                Line lk = forw_lines.at(k);

                // condition 1
                Point2f d = lk.Center - lc.Center;
                float dist = d.dot(d);
                if( dist > dth) continue;  //
                // condition 2
                float delta_theta1 = fabs(lk.theta - lc.theta);
                float delta_theta2 = 3.1415926 - delta_theta1;
                if( delta_theta1 < th || delta_theta2 < th)
                {
                    //std::cout << "theta: "<< lf.theta * 180 / 3.14259 <<" "<< lc.theta * 180 / 3.14259<<" "<<delta_theta1<<" "<<delta_theta2<<std::endl;
                    //candidate.push_back(lk);
                    //float cost = fabs(lk.image_dx - lc.image_dx) + fabs( lk.image_dy - lc.image_dy) + dist;
                    float cost = fabs(lk.line_grad_avg - lc.line_grad_avg) + dist/10.0;

                    if(cost < grad_err_min_i)
                    {
                        grad_err_min_i = cost;
                        best_i = k;
                    }
                }

            }
        }

        if( grad_err_min_i < 50 && best_i == i){

            //std::cout<< "line match cost: "<<grad_err_min_j<<" "<<grad_err_min_i <<"\n\n";
            lineMatches.push_back(make_pair(best_j,i));
        }
        /*
        vector<Line> l;
        l.push_back(lf);
        vector<Line> best;
        best.push_back(best_match);
        visualizeLineTrackCandidate(l,forwframe_->img,"forwframe_");
        visualizeLineTrackCandidate(best,curframe_->img,"curframe_best");
        visualizeLineTrackCandidate(candidate,curframe_->img,"curframe_");
        cv::waitKey(0);
        */
    }

}

//#define NLT
#ifdef  NLT
void LineFeatureTracker::readImage(const cv::Mat &_img)
{
    cv::Mat img;
    TicToc t_p;
    frame_cnt++;
    cv::remap(_img, img, undist_map1_, undist_map2_, CV_INTER_LINEAR);
    //ROS_INFO("undistortImage costs: %fms", t_p.toc());
    if (EQUALIZE)   // 直方图均衡化
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(img, img);
    }

    bool first_img = false;
    if (forwframe_ == nullptr) // 系统初始化的第一帧图像
    {
        forwframe_.reset(new FrameLines);
        curframe_.reset(new FrameLines);
        forwframe_->img = img;
        curframe_->img = img;
        first_img = true;
    }
    else
    {
        forwframe_.reset(new FrameLines);  // 初始化一个新的帧
        forwframe_->img = img;
    }

    // step 1: line extraction
    TicToc t_li;
    int lineMethod = 2;
    bool isROI = false;
    lineDetector ld(lineMethod, isROI, 0, (float)img.cols, 0, (float)img.rows);
    //ROS_INFO("ld inition costs: %fms", t_li.toc());
    TicToc t_ld;
    forwframe_->vecLine = ld.detect(img);

    for (size_t i = 0; i < forwframe_->vecLine.size(); ++i) {
        if(first_img)
            forwframe_->lineID.push_back(allfeature_cnt++);
        else
            forwframe_->lineID.push_back(-1);   // give a negative id
    }
    ROS_INFO("line detect costs: %fms", t_ld.toc());

    // step 3: junction & line matching
    if(curframe_->vecLine.size() > 0)
    {
        TicToc t_nlt;
        vector<pair<int, int> > linetracker;
        NearbyLineTracking(forwframe_->vecLine, curframe_->vecLine, linetracker);
        ROS_INFO("line match costs: %fms", t_nlt.toc());

        // 对新图像上的line赋予id值
        for(int j = 0; j < linetracker.size(); j++)
        {
            forwframe_->lineID[linetracker[j].second] = curframe_->lineID[linetracker[j].first];
        }

        // show NLT match
        visualizeLineMatch(curframe_->vecLine, forwframe_->vecLine, linetracker,
                           curframe_->img, forwframe_->img, "NLT Line Matches", 10, true,
                           "frame");
        visualizeLinewithID(forwframe_->vecLine,forwframe_->lineID,forwframe_->img,"forwframe_");
        visualizeLinewithID(curframe_->vecLine,curframe_->lineID,curframe_->img,"curframe_");
        stringstream ss;
        ss <<"/home/hyj/datasets/line/" <<frame_cnt<<".jpg";
        // SaveFrameLinewithID(forwframe_->vecLine,forwframe_->lineID,forwframe_->img,ss.str().c_str());
        waitKey(5);


        vector<Line> vecLine_tracked, vecLine_new;
        vector< int > lineID_tracked, lineID_new;
        // 将跟踪的线和没跟踪上的线进行区分
        for (size_t i = 0; i < forwframe_->vecLine.size(); ++i)
        {
            if( forwframe_->lineID[i] == -1)
            {
                forwframe_->lineID[i] = allfeature_cnt++;
                vecLine_new.push_back(forwframe_->vecLine[i]);
                lineID_new.push_back(forwframe_->lineID[i]);
            }
            else
            {
                vecLine_tracked.push_back(forwframe_->vecLine[i]);
                lineID_tracked.push_back(forwframe_->lineID[i]);
            }
        }
        int diff_n = 30 - vecLine_tracked.size();  // 跟踪的线特征少于50了，那就补充新的线特征, 还差多少条线
        if( diff_n > 0)    // 补充线条
        {
            for (int k = 0; k < vecLine_new.size(); ++k) {
                vecLine_tracked.push_back(vecLine_new[k]);
                lineID_tracked.push_back(lineID_new[k]);
            }
        }

        forwframe_->vecLine = vecLine_tracked;
        forwframe_->lineID = lineID_tracked;

    }
    curframe_ = forwframe_;
}
#endif

#define MATCHES_DIST_THRESHOLD 30

void LineFeatureTracker::visualize_line_match(Mat imageMat1, Mat imageMat2,
                          std::vector<KeyLine> octave0_1)
{
    //	Mat img_1;
    ROS_INFO("VISUALIZE .........................................");
    cv::Mat img1, img2;
    if (imageMat1.channels() != 3)
    {
        cv::cvtColor(imageMat1, img1, cv::COLOR_GRAY2BGR);
    }
    else
    {
        img1 = imageMat1;
    }

    if (imageMat2.channels() != 3)
    {
        cv::cvtColor(imageMat2, img2, cv::COLOR_GRAY2BGR);
    }
    else
    {
        img2 = imageMat2;
    }

    //    srand(time(NULL));
    int lowest = 0, highest = 255;
    int range = (highest - lowest) + 1;
    for (int k = 0; k < octave0_1.size(); ++k)
    {

        unsigned int r = lowest + int(rand() % range);
        unsigned int g = lowest + int(rand() % range);
        unsigned int b = lowest + int(rand() % range);

        cv::Point startPoint = cv::Point(int(octave0_1[k].startPointX), int(octave0_1[k].startPointY));
        cv::Point endPoint = cv::Point(int(octave0_1[k].endPointX), int(octave0_1[k].endPointY));
        
        double len = std::min(1.0, 1.0 * linetrack_cnt[k] / WINDOW_SIZE);

        cv::line(img1, startPoint, endPoint, cv::Scalar(255 * (1 - len), 0, 255 * len), 2, 8);
    }
    /* plot matches */
    /*
    cv::Mat lsd_outImg;
    std::vector<char> lsd_mask( lsd_matches.size(), 1 );
    drawLineMatches( imageMat1, octave0_1, imageMat2, octave0_2, good_matches, lsd_outImg, Scalar::all( -1 ), Scalar::all( -1 ), lsd_mask,
    DrawLinesMatchesFlags::DEFAULT );

    imshow("LSD matches", lsd_outImg );
    */
    imshow("LSD matches1", img1);
    waitKey(5);
}

void visualize_line_match(Mat imageMat1, Mat imageMat2,
                          std::vector<KeyLine> octave0_1, std::vector<KeyLine>octave0_2,
                          std::vector<bool> good_matches)
{
    //	Mat img_1;
    cv::Mat img1,img2;
    if (imageMat1.channels() != 3){
        cv::cvtColor(imageMat1, img1, cv::COLOR_GRAY2BGR);
    }
    else{
        img1 = imageMat1;
    }

    if (imageMat2.channels() != 3){
        cv::cvtColor(imageMat2, img2, cv::COLOR_GRAY2BGR);
    }
    else{
        img2 = imageMat2;
    }

    //    srand(time(NULL));
    int lowest = 0, highest = 255;
    int range = (highest - lowest) + 1;
    for (int k = 0; k < good_matches.size(); ++k) {

        if(!good_matches[k]) continue;

        KeyLine line1 = octave0_1[k];  // trainIdx
        KeyLine line2 = octave0_2[k];  //queryIdx

        unsigned int r = lowest + int(rand() % range);
        unsigned int g = lowest + int(rand() % range);
        unsigned int b = lowest + int(rand() % range);
        cv::Point startPoint = cv::Point(int(line1.startPointX), int(line1.startPointY));
        cv::Point endPoint = cv::Point(int(line1.endPointX), int(line1.endPointY));
        cv::line(img1, startPoint, endPoint, cv::Scalar(r, g, b),2 ,8);

        cv::Point startPoint2 = cv::Point(int(line2.startPointX), int(line2.startPointY));
        cv::Point endPoint2 = cv::Point(int(line2.endPointX), int(line2.endPointY));
        cv::line(img2, startPoint2, endPoint2, cv::Scalar(r, g, b),2, 8);
        cv::line(img2, startPoint, startPoint2, cv::Scalar(0, 0, 255),1, 8);
        cv::line(img2, endPoint, endPoint2, cv::Scalar(0, 0, 255),1, 8);

    }
    /* plot matches */
    /*
    cv::Mat lsd_outImg;
    std::vector<char> lsd_mask( lsd_matches.size(), 1 );
    drawLineMatches( imageMat1, octave0_1, imageMat2, octave0_2, good_matches, lsd_outImg, Scalar::all( -1 ), Scalar::all( -1 ), lsd_mask,
    DrawLinesMatchesFlags::DEFAULT );

    imshow( "LSD matches", lsd_outImg );
    */
    imshow("LSD matches1", img1);
    imshow("LSD matches2", img2);
    waitKey(1);
}

void LineFeatureTracker::reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < status.size(); i+=2)
        if (status[i] && status[i+1])
            v[j++] = v[i/2];
    v.resize(j);
}

void LineFeatureTracker::reduceVector(vector<Line2D> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < status.size(); i+=2)
        if (status[i] && status[i+1])
            v[j++] = v[i/2];
    v.resize(j);
}

void LineFeatureTracker::reduceVector(vector<cv::line_descriptor::KeyLine> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < status.size(); i+=2)
        if (status[i] && status[i+1])
            v[j++] = v[i/2];
    v.resize(j);
}

void LineFeatureTracker::reduceVector(vector<Line> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < status.size(); i+=2)
        if (status[i] && status[i+1])
            v[j++] = v[i/2];
    v.resize(j);
}


void LineFeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {

        // ROS_INFO("in add points-------------------------------------");
        //pushback keylsd;     
        cv::line_descriptor::KeyLine keyL;
        keyL.endPointX = p.endPixelCoord.x;
        keyL.endPointY = p.endPixelCoord.y;

        keyL.startPointX = p.startPixelCoord.x;
        keyL.startPointY = p.startPixelCoord.y;

        forwframe_->keylsd.push_back(keyL);


        //pushback vecline:
        Line l;
        l.StartPt = keyL.getStartPoint(); // check if this actually works!
        l.EndPt = keyL.getEndPoint();     // check if this actually works!
        // l.length = lsd.lineLength;       // NEED TO DO!!!

        forwframe_->vecLine.push_back(l);

        forwframe_->lineID.push_back(-1);

        ids.push_back(-1);       // Feature point id, initially assign a value of -1 to these new feature points, and use a global variable in the updateID() function to assign values ​​to him
        
        // track_cnt.push_back(1);  // Number of observations to initialize feature points: 1 time
        linetrack_cnt.push_back(1);
    
    }
}

bool LineFeatureTracker::updateID(unsigned int i)
{

    if (i < ids.size())
        {
            if (ids[i] == -1)
                ids[i] = n_id++;   // n_id is a global variable, giving each feature point a unique id
            return true;
        }
        else
            return false;

    // if (i < curframe_->lineID.size())
    // {
    //     if (curframe_->lineID[i] == -1)
    //         curframe_->lineID[i] = n_id++;   // n_id is a global variable, giving each feature point a unique id
    //     return true;
    // }
    // else
    //     return false;
}

bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}


void LineFeatureTracker::readImage(const cv::Mat &_img)
{

    if (init) {
        init = false;
        forwframe_.reset(new FrameLines);
        curframe_.reset(new FrameLines);
        prevframe_.reset(new FrameLines);
    }

    cv::Mat img;
    TicToc t_r;
    // ROS_INFO("IN READ IMAGE LINE [1]");


    cv::remap(_img, img, undist_map1_, undist_map2_, INTER_LINEAR);

    if (EQUALIZE)   // Histogram equalization
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(img, img);
    }
    else
        img = _img;

    // ROS_INFO("IN READ IMAGE LINE [1.2]");

    if (forwframe_->img.empty())
    {
        // ROS_INFO("IN READ IMAGE LINE [1.3]");
        forwframe_.reset(new FrameLines);
        curframe_.reset(new FrameLines);
        //potentially also prevframe^^??
        // ROS_INFO("IN READ IMAGE LINE [1.4]");

        prevframe_->img = curframe_->img = forwframe_->img = img;
        
    }
    else
    {
        // ROS_INFO("IN READ IMAGE LINE [1.5]");

        forwframe_.reset(new FrameLines);
        forwframe_->img = img;
    }

    // ROS_INFO("IN READ IMAGE LINE [2]");

    forwframe_->keylsd.clear();


    if (curframe_->keylsd.size() > 0)       // i时刻的 特征点
    {

        visualize_line_match(curframe_->img.clone(), curframe_->img.clone(), curframe_->keylsd);

        // ROS_INFO("IN OPTICAL FLOW [1]");

        // ROS_INFO("PREV KELSD SIZE; %i: ", curframe_->lineID.size());

        std::vector<cv::Point2f> prevPoints, forwardedPoints, prevPointsReversed;

        for (const auto &l : curframe_->keylsd)
        {
            prevPoints.emplace_back(l.getStartPoint());
            prevPoints.emplace_back(l.getEndPoint());
        }

        // ROS_INFO("IN OPTICAL FLOW [2]");

        // FLD TO OPTICAL FLOW POINTS
        std::vector<uchar> status, statusReverse;
        std::vector<float> err, errReverse;
        cv::Size windowSize(21, 21);

        cv::calcOpticalFlowPyrLK(curframe_->img,
                                 forwframe_->img,
                                 prevPoints,
                                 forwardedPoints,
                                 status,
                                 err,
                                 cv::Size(21, 21),
                                 3);

        cv::calcOpticalFlowPyrLK(forwframe_->img,
                                 curframe_->img,
                                 forwardedPoints,
                                 prevPointsReversed,
                                 statusReverse,
                                 errReverse,
                                 cv::Size(21, 21),
                                 3);

        // ROS_INFO("IN OPTICAL FLOW [3]");
        vector<KeyLine> vecLine_tracked;

        for (std::size_t idx = 0; idx < forwardedPoints.size(); idx += 2)
        {
            Line2D l;
            cv::line_descriptor::KeyLine keyL;
            l.startPixelCoord = forwardedPoints[idx];
            l.endPixelCoord = forwardedPoints[idx + 1];
            m_forwardedLines.emplace_back(l);

            keyL.endPointX = l.endPixelCoord.x;
            keyL.endPointY = l.endPixelCoord.y;

            keyL.startPointX = l.startPixelCoord.x;
            keyL.startPointY = l.startPixelCoord.y;

            forwframe_->keylsd.emplace_back(keyL);

            Line lineV;
            lineV.StartPt = keyL.getStartPoint(); // check if this actually works!
            lineV.EndPt = keyL.getEndPoint();     // check if this actually works!
        // l.length = lsd.lineLength;       // NEED TO DO!!!

            forwframe_->vecLine.push_back(lineV);
        }




        // ROS_INFO("IN OPTICAL FLOW [4]");
        // if reverse optical flow from forwarded points
        // do not match the previous points, then mark the point as untracked
        for (std::size_t idx = 0; idx < prevPoints.size(); idx++)
        {
            const auto &p1 = prevPoints[idx];
            const auto &p2 = prevPointsReversed[idx];
            // ROS_INFO("???????????????????/pixel errow optical flow: %f", std::abs(p1.x - p2.x));

            if (std::abs(p1.x - p2.x) > 3     // 3 CAN BE A DIFFERENT VALUE!!!
                || std::abs(p1.y - p2.y) > 3 || !inBorder(forwardedPoints[idx])) // 3 CAN BE A DIFFERENT VALUE!!!
            {

                // ROS_INFO("+++++++++++++++pixel errow optical flow: %f", std::abs(p1.x - p2.x));
                status[idx] = static_cast<uchar>(0);
            }
        }
        

        
        // ROS_INFO("IN READ IMAGE LINE [3]");


        // prev_pts is a vector container, status indicates whether each point is tracked
        //successfully, reduceVector puts all the tracked points in front of this vector
        //container, such as 1,0,0,1,1,0 becomes 1 ,1,1        
        

        //CHECK THIS!!!!!!!????????

        reduceVector(prevframe_->keylsd, status);
        reduceVector(curframe_->keylsd, status);
        reduceVector(forwframe_->keylsd, status);

        reduceVector(prevframe_->vecLine, status);
        reduceVector(curframe_->vecLine, status);
        reduceVector(forwframe_->vecLine, status);

        // reduceVector(curframe_->lineID, status);
        reduceVector(ids, status);
        reduceVector(linetrack_cnt, status);

        // curframe_->keylsd = forwframe_->keylsd;
        // curframe_->vecLine = forwframe_->vecLine;






    }

    if (PUB_THIS_FRAME)
    {
        // rejectWithF();              // Exclude outliers by computing the F matrix

        for (auto &n : linetrack_cnt)   // Update the number of tracking frames of the feature on tracking, successfully tracked from frame i to frame i+1, and the number of tracking frames +1
            n++;

        // ROS_DEBUG("set mask begins");
        // TicToc t_m;
        // setMask();                 // Set the template to cover up the areas where feature points have been detected, and other areas are used to detect new feature points
        // ROS_DEBUG("set mask costs %fms", t_m.toc());

        // forwframe_->lineID = prevframe_->lineID;

        // ROS_INFO("detect feature begins");
        TicToc t_t;    //5 IS ARBITRARY - SHOULD CHANGE
        int n_max_cnt = 10 - static_cast<int>(forwframe_->keylsd.size());  // If the number of current feature points < MAX_CNT, then detect some new feature points
        if (n_max_cnt > 0)    // is less than the maximum number of feature points, then add new features
        {
            
            std::vector<KeyLine> lsd, keylsd;

            int length_threshold = 80;
            float distance_threshold = 1.41421356f;
            double canny_th1 = 50.0;
            double canny_th2 = 50.0;
            int canny_aperture_size = 3;
            bool do_merge = false;
            Ptr<cv::ximgproc::FastLineDetector> fld_ = cv::ximgproc::createFastLineDetector(length_threshold,
                                                                                            distance_threshold, canny_th1, canny_th2, canny_aperture_size, do_merge);
            vector<Vec4f> fld;

            fld_->detect(img, fld); 

            // Line2D l;
            // l.startPixelCoord.x = fld[0](0);
            // l.startPixelCoord.y = fld[0](1);
            // l.endPixelCoord.x = fld[0](2);
            // l.endPixelCoord.y = fld[0](3);

            // n_pts.emplace_back(l);
            
            for (auto &dLine : fld)
            {
                Line2D l;
                l.startPixelCoord.x = dLine(0);
                l.startPixelCoord.y = dLine(1);
                l.endPixelCoord.x = dLine(2);
                l.endPixelCoord.y = dLine(3);

                n_pts.emplace_back(l);
            }
            
        }
        else {
            n_pts.clear();
        }

        // ROS_INFO("add feature begins");
        TicToc t_a;
        addPoints();         // Add this new feature point to forw_pts
        
        prevframe_->keylsd = forwframe_->keylsd;
        prevframe_->vecLine = forwframe_->vecLine;
        prevframe_->lineID = forwframe_->lineID;
        prevframe_ = forwframe_;

    }

    
    curframe_->keylsd = forwframe_->keylsd;
    curframe_->vecLine = forwframe_->vecLine;
    curframe_->lineID = forwframe_->lineID;
    curframe_ = forwframe_;


}