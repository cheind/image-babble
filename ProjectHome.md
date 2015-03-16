**Note**
Due to Google Code is shutting down this project has moved to
https://github.com/cheind/image-babble

ImageBabble is a lightweight C++ library to send and receive images via networks. It provides implementations of fast and reliable communication protocols.

The minimum example below shows the essence of an image streaming server. It uses OpenCV for image and device handling. ImageBabble itself does not dependent on OpenCV, but its design allows smooth interaction between libraries.

```
// Image streaming server

namespace ib = ::imagebabble;

ib::fast_server< ib::image > is;
is.startup("tcp://*:6000");

cv::VideoCapture vc(0);
cv::Mat cv_img;

while (vc.grab() && vc.retrieve(cv_img)) 
{
  ib::image ib_img;
  ib::cvt_image(cv_img, ib_img, ib::copy_mem());

  is.publish(ib_img);
}
```

The following snippet shows the client implementation

```
// Image streaming client

namespace ib = ::imagebabble;

ib::fast_client< ib::image > ic;
ic.startup("tcp://127.0.0.1:6000");

ib::image ib_image;
while (ic.receive(ib_image, 5000)) {

  cv::Mat cv_img;
  ib::cvt_image(ib_image, cv_img, ib::copy_mem());

  cv::imshow("image", cv_img);
  cv::waitKey(1);
}
```
