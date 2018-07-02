
// Copy from Opencv example 

/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2018, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#include "train.h"

int main( int argc, char** argv )
{
    String pos_dir = "../../../../small_armor_4_selected";
    String neg_dir = "../../../../big_armor_neg";
    String test_dir = "../../test-armor";
    String svm_file = "../output/armor_model.yml";
    String obj_det_filename = "../output/armor_descriptor.yml";
    String videofilename = "../../../video/armor_small.avi";
    bool test_detector = false;
    bool train_twice = false;
    bool flip_samples = false; // 镜像反转 增加一倍的训练量

    if ( test_detector )
    {
        test_trained_detector( obj_det_filename, test_dir, videofilename );
        exit( 0 );
    }
    
    vector< Mat > positive_lst,   // 正样本
                  full_neg_lst,   // 负样本
                  neg_lst,        // 负样本经过处理的 
                  gradient_lst;   // 正样本的HOG

    vector< int > labels;         // 标签

    clog << "Positive images are being loaded..." ;
    load_images( pos_dir, positive_lst, true); // 加载正样本, 显示中间输出
    if ( positive_lst.size() > 0 )
    {
        clog << "...[done]" << endl;
    }
    else{
        clog << "no image in " << pos_dir <<endl;
        return 1;
    }
    Size pos_image_size = positive_lst[0].size();

    for ( size_t i = 0; i < positive_lst.size(); ++i )
    {
        if( positive_lst[i].size() != pos_image_size )
        {
            cout << "All positive images should be same size!" << endl;
            exit( 1 );
        }
    }
    pos_image_size = pos_image_size / 8 * 8;

    clog << "Negative images are being loaded...";
    load_images( neg_dir, full_neg_lst, true );  // 加载负样本, 显示中间输出
    sample_neg( full_neg_lst, neg_lst, pos_image_size );  // 让负样本和正样本保持尺寸一致
    clog << "...[done]" << endl;
    
    clog << "Histogram of Gradients are being calculated for positive images...";
    computeHOGs( pos_image_size, positive_lst, gradient_lst, false ); // 正样本不镜像反转
    size_t positive_count = gradient_lst.size();

    labels.assign( positive_count, +1 );  // 正样本 responce = +1

    clog << "...[done] ( positive count : " << positive_count << " )" << endl;
    clog << "Histogram of Gradients are being calculated for negative images...";
    computeHOGs( pos_image_size, neg_lst, gradient_lst, false ); // 负样本进行翻转加倍
    size_t negative_count = gradient_lst.size() - positive_count;  // 从第几个开始就是负样本

    labels.insert( labels.end(), negative_count, -1 ); // 负样本 response = -1

    CV_Assert( positive_count < labels.size() );
    clog << "...[done] ( negative count : " << negative_count << " )" << endl;
    Mat train_data;
    convert_to_ml( gradient_lst, train_data );  // 将数据转换成ml训练的格式
    clog << "Training SVM...";
    Ptr< SVM > svm = SVM::create();
    /* Default values to train SVM */
    svm->setCoef0( 0.0 );
    svm->setDegree( 3 );
    svm->setTermCriteria( TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 1000, 1e-3 ) );
    svm->setGamma( 0 );
    svm->setKernel( SVM::LINEAR );
    svm->setNu( 0.5 );
    svm->setP( 0.1 ); // for EPSILON_SVR, epsilon in loss function?
    svm->setC( 0.01 ); // From paper, soft classifier
    svm->setType( SVM::EPS_SVR ); // C_SVC; // EPSILON_SVR; // may be also NU_SVR; // do regression task
    svm->train( train_data, ROW_SAMPLE, labels );
    clog << "...[done]" << endl;
    svm->save(svm_file);
    HOGDescriptor hog;
    hog.winSize = pos_image_size;
    hog.setSVMDetector( get_svm_detector( svm ) );
    hog.save(obj_det_filename);

    test_trained_detector( obj_det_filename, test_dir, videofilename );
    return 0;
}