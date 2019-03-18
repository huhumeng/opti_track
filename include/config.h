/*******************************************************
 * Copyright (C) 2019, lingy17@mails.tsinghua.edu.cn
 * 
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/


#pragma once

#include <iostream>
#include <memory>

#include <opencv2/core/core.hpp>

class Config{

    typedef std::shared_ptr<Config> Ptr;

public:

    ~Config(){
        if(config_ != nullptr)
            fileStorage_.release();
    }

    template <typename T>
    static T getParam(const std::string& key){

        return (T)config_->fileStorage_[key];
    }
    
    static void setParameterFile(const std::string& filename){
        if(config_ == nullptr)
            config_ = Ptr(new Config);
        
        bool succ = config_->fileStorage_.open(filename, cv::FileStorage::READ);

        if(!succ){
            std::cerr << "Open file " << filename << " failed!" << std::endl;
            exit(-1);
        }
    }


private:

    Config(){}
    Config(const Config& other) = delete;
    Config operator=(const Config& other) = delete;

    cv::FileStorage fileStorage_;

    static Ptr config_;
};