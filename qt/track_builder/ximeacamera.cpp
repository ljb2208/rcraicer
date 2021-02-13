#include "ximeacamera.h"
#include <cstring>


XimeaCamera::XimeaCamera(std::string serialNumber)
{
    this->serialNumber = serialNumber;
    xiH = NULL;
    memset(&ximage, 0, sizeof(ximage));
    ximage.size = sizeof(XI_IMG);
}

bool XimeaCamera::open()
{
    XI_RETURN ret = xiOpenDeviceBy(XI_OPEN_BY_SN, serialNumber.c_str(), &xiH);

    if (ret == XI_OK)
    {
        applySettings();
        ret = xiStartAcquisition(xiH);

        if (ret == XI_OK)
            return true;
        else
            return false;
    }

    return false;
}

void XimeaCamera::close()
{
    if (xiH != NULL)
    {
        xiCloseDevice(xiH);
        xiH = NULL;
    }
}

bool XimeaCamera::getImage(cv::Mat& image)
{
    XI_RETURN ret = xiGetImage(xiH, 1000, &ximage);

    if (ret == XI_OK)
    {
        image.create(ximage.height, ximage.width, CV_8UC3);
        image.data = (uchar*) ximage.bp;
        return true;
    }

    return false;
}

void XimeaCamera::applySettings()
{
    xiSetParamInt(xiH, XI_PRM_DOWNSAMPLING, XI_DWN_1x1);
    xiSetParamInt(xiH, XI_PRM_IMAGE_DATA_FORMAT, XI_RGB24);
    xiSetParamInt(xiH, XI_PRM_AEAG, XI_ON);
    xiSetParamInt(xiH, XI_PRM_AUTO_WB, XI_ON);
}
