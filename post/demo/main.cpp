
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;


struct Line{
    Point A, B;
    float angle;
};

template <typename T>
void combine_inner(T &data, int start, int n, int m, int depth, T temp, std::vector<T> &result)
{
    if (depth == m - 1)
    {
        //最内层循环 将temp加入result
        for (int i = start; i < n - (m - depth - 1); ++i)
        {
            temp[depth] = data[i];
            result.emplace_back(temp);
        }
    }
    else
        for (int i = start; i < n - (m - depth - 1); ++i)
        {
            temp[depth] = data[i];//每层输出一个元素
            combine_inner(data, i + 1, n, m, depth + 1, temp, result);
        }
}

template <typename T>
std::vector<T> combine(T& data, int m) {
    if (m <= 0) return {};
    int depth = 0;
    std::vector<T> result;
    T temp(m);
    combine_inner(data, 0, data.size(), m, depth, temp, result);
    return std::move(result);
}

void minboundquad(vector<cv::Point>& pts, vector<Point>& res)
{
    if(pts.size() <= 3){
        for (int i = 0; i < 4; ++i) {
            res.push_back(pts[i % pts.size()]);
        }
        return;
    }
    Mat dst;
    convexHull(pts, dst, true, true);
    if(dst.rows == 3){
        res.push_back(dst.at<Point>(0, 0));
        res.push_back(dst.at<Point>(1, 0));
        res.push_back(dst.at<Point>(2, 0));
        res.push_back(dst.at<Point>(2, 0));
        return;
    } else if(dst.rows == 4){
        res.push_back(dst.at<Point>(0, 0));
        res.push_back(dst.at<Point>(1, 0));
        res.push_back(dst.at<Point>(2, 0));
        res.push_back(dst.at<Point>(2, 0));
        return;
    }
    vector<Line> l;
    //获取所有的边
    for (int j = 0; j < dst.rows; ++j) {
        l.push_back({dst.at<Point>(j, 0), dst.at<Point>((j+1)%dst.rows, 0)});
        l[j].angle = atan2(l[j].A.y - l[j].B.y, l[j].A.x - l[j].B.x);
    }

    auto r = combine(l, 4);
    //考虑所有的情况
    for (int k = 0; k < r.size(); ++k) {

    }
}

void drawPoly()
{
    Mat img(600, 600, CV_8U, Scalar(0));
    Point points[2][4];
    points[0][0] = Point(100, 115);
    points[0][1] = Point(255, 135);
    points[0][2] = Point(140, 365);
    points[0][3] = Point(100, 300);
    points[1][0] = Point(300, 315);
    points[1][1] = Point(555, 335);
    points[1][2] = Point(340, 565);
    points[1][3] = Point(300, 500);
    //ppt[]要同时添加两个多边形顶点数组的地址头
    const Point* pts[] = {points[0],points[1]};
    //npts[]要定义每个多边形的定点数
    int npts[] = {4,4};
    polylines(img, pts, npts,2,true,Scalar(255),5,8,0);
    namedWindow("Poly");
    imshow("Poly", img);
    waitKey();
    fillPoly(img,pts,npts,2,Scalar(255),8,0,Point());
    imshow("Poly", img);
    waitKey();
}

int main() {
    vector<vector<cv::Point> > shape = {
            {{200, 200},{500, 200},{650, 500},{600, 600},{100, 650}}
    };
    cv::Mat dst = Mat::zeros(cv::Size(800, 800), CV_8UC3);
    polylines(dst, shape, true ,Scalar(0,255,255),1,8,0);

    vector<Point> res;
    minboundquad(shape[0], res);

    cv::imshow("image", dst);
    cv::waitKey(0);

//    const cv::Point** p = reinterpret_cast<const cv::Point **>(&shape);
//    cv::Mat dst = cv::Mat(cv::Size(20, 20), CV_8UC3);
//    cv::polylines(dst, p, reinterpret_cast<const int *>(5), 1, true, cv::Scalar(0, 0, 255));
//
//    cv::imshow("image", dst);
//    cv::waitKey(0);
//    //minboundquad(shape, dst);
//    std::cout << "Hello, World!" << std::endl;
    return 0;
}
