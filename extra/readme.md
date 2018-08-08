## extra模块
这是一个曾经迭代版本的装甲识别算法,删减了部分检测算法,使得它可以在高识别armor的同时,误识别很多作为负样本进行训练。armor_sample和train的用法流程都是一样的。
```shell
mkdir build
cd build
cmake ..
make
```
## armor_sample
这是一份阉割的装甲识别，没有误识别筛选、没有直方图匹配、没有SVM、只有单纯的detect。为了方便，armor_sample通过读取视频来获得样本。

在`armor_sample/detect.cpp`中700多行
```c++
if (makeRectSafe(target,img.size()) == true)
{
	cv::Mat armor_roi = img(target);
	cv::resize(armor_roi, armor_roi, cv::Size(100,25));
	//cv::resize(armor_roi, armor_roi, cv::Size(60,25));
	char str[100];
    sprintf(str, "../../../../sample/armor_2/armor_%d.jpg", img_idx++);
	cv::imwrite(str, armor_roi);		
}
```
其中，cv::resize要把大装甲和小装甲处理成不同的尺寸，大装甲Size(100,25)，小装甲Size(60,25)
```c++
cv::resize(armor_roi, armor_roi, cv::Size(100,25));  // 大装甲
cv::resize(armor_roi, armor_roi, cv::Size(60,25));   // 小装甲		
```
生成的样本要放在一个指定的路径，比如这里的设置是video/是视频路径，sample/是样本路径。
## train
顾名思义，这是训练的模块，代码是从opencv example中修改的。在main.cpp中，训练过程只需要指定pos_dir正样本文件夹和neg_dir负样本文件夹，armor_sample是个识别率很高并且误识别也不低的算法（大约3：1吧），大多数误识别都是两块装甲板的之间。刚好可以获得足够的正负样本拿来训练。
```c++
String pos_dir = "../../../../small_armor_4_selected";
String neg_dir = "../../../../big_armor_neg";
String test_dir = "../../test-armor";
String svm_file = "../output/armor_model.yml";
String obj_det_filename = "../output/armor_descriptor.yml";
String videofilename = "../../../video/armor_small.avi";
```
而其他的都是opencv example自带的测试需要的代码，我并没有删除。最后我们只需要一个svm_file，并把它拷贝到config/文件夹下，需要小装甲和大装甲svm_file, 在`/include/detect_factary/armor_detect.hpp`中加载了这两个svm_file，加载svm还是比较长，但是检测耗时还能接受。
```c++
class ArmorDetector
{
public:
    ArmorDetector() {
		init_armorHist("../config/armor2ev0.jpg", "../config/armor2ev-3.jpg"); // load big armor
		svm_big = StatModel::load<SVM>("../config/big_armor_model.yml");
		svm_small = StatModel::load<SVM>("../config/armor_model.yml");
	};
	... ...
};
```
`armor_descriptor.yml`是`extra/trian`代码本身测试用的，我们在装甲识别用不到它。`/config/big_armor_model.yml`和`/config/armor_model.yml`这两个文件要自己训练并放到config/文件夹下面，然后程序才能跑起来。

