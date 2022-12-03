/*-------------------------Использовал для отладки до начала написания GUI---------------
std::vector<cv::Point2i> vec;
void mouse_callback(int  event, int  x, int  y, int  flag, void *param)
{
if (event == cv::MouseEventTypes::EVENT_LBUTTONDOWN)
{
    // Store point coordinates
    vec.emplace_back(x, y);
}
}
*/
int main() {

/*-------------------------Использовал для отладки до начала написания gui--------------------
cv:: Mat internal = (cv::Mat_<double>(3, 3) << 50, 0, 960, 0, 50, 540, 0, 0, 1);
auto kek = std::make_shared<models::plane>(cv::Vec3d(0, 0, 0), 10, 50);
auto newCamera = models::camera(internal);
newCamera.rotate(CV_PI, xAxis);
newCamera.move(cv::Vec3d(0, 0, 10));
auto strategy = std::make_shared<models::ParabolaStrategy>();
kek->setStrategy(strategy);
auto flag = false;

while(true) {
    newCamera.clear();
    newCamera.displayModelPoints(kek);
    auto lol = newCamera.getCameraPlane();
    if (flag) {
    newCamera.displayPoints(kek->getLeftBorder());
    newCamera.displayPoints(kek->getRightBorder());}
    cv::namedWindow("img", 1);
    cv::setMouseCallback("img", mouse_callback);
    imshow("img", lol);
    if (vec.size() == 3) {
        flag = true;
        auto puk = newCamera.reverseProject(kek, vec);
        kek->addBorders(puk);
        newCamera.displayPoints(kek->getLeftBorder());
        newCamera.displayPoints(kek->getRightBorder());
        vec.erase(vec.begin(), std::next(vec.begin(), 3));
        imshow("img", lol);
    }

    int a = cv::waitKey(0);
    std::cout<<a;
    if (a == 27) {
        break;
    }
    if (a == 122) {
        newCamera.move(cv::Vec3d(0, 1, 0));
        for (int i = 0; i < vec.size(); i++) {
            vec[i].y += 1;
        }
        continue;
    }
    if (a == 120) {
        newCamera.move(cv::Vec3d(0, -20, 0));
        for (int i = 0; i < vec.size(); i++) {
            vec[i].y -= 20;
        }
        continue;
    }
    else {
        lol.at<uchar>(vec.back().y, vec.back().x) = 255;
    }
}

newCamera.rotate(-CV_PI/3, xAxis);
newCamera.setCoordinatesOfCenter(cv::Vec3d(0, -60, 10));
std::cout<<newCamera.getCoordinatesOfCenter();
while(true) {
    newCamera.clear();
    newCamera.displayModelPoints(kek);
    newCamera.displayPoints(kek->getRightBorder());
    newCamera.displayPoints(kek->getLeftBorder());
    imshow("img", newCamera.getCameraPlane());
    int a = cv::waitKey(0);
    std::cout<<a;
    if (a == 27) {
        break;
    }
    if (a == 122) {
        newCamera.move(cv::Vec3d(0, 1, 0));
    }
    if (a == 120) {
        newCamera.move(cv::Vec3d(0, -1, 0));
    }
}
*/
}

