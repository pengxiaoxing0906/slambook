//
// Created by pxx on 4/24/19.
//

#include "../include/myslam/config.h"
namespace myslam
{
    void Config::setParameterFile(const std::string& filename)
    {
        if(config_== nullptr)
            config_=shared_ptr<Config>(new Config);  //[3]不懂写法 知道是创建一个指针的意思
        config_->file_=cv::FileStorage(filename.c_str(),cv::FileStorage::READ);
        cout<<"filename path"<<filename<<endl;
        if(config_->file_.isOpened()==false)
        {
            std::cerr<<"parameter file "<<filename<<" does not exist."<<endl;
            config_->file_.release();
            return;
        }

    }
    Config::~Config()
    {
        if(file_.isOpened())
        {
            file_.release();
        }
    }

    shared_ptr<Config>Config::config_= nullptr; //【4】不知道这一句的写法 以及放在这干嘛的
}
