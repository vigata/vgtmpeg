## About

vgtmpeg is a ffmpeg/avconv clone that adds a number of additional features to the stock ffmpeg and libavformat/libavcodec libraries.

## vgtmpeg vs ffmpeg

We love ffmpeg for its versatility and wealth of contributor support. vgtmpeg doesn’t try to be a substitute to ffmpeg. Rather, we try to have vgtmpeg to be a specialized ffmpeg distribution with added features. Maybe some of those features will make it into the ffmpeg.

## DVD support

vgtmpeg adds support for DVD in its version of libavformat. DVD support is implemented by adding a new ‘dvdurl’ protocol that can parse DVD folders, DVD ISO files, DVD devices and more. All the regular features available in vgtmpeg/ffmpeg are still available when a dvd url is used. From direct stream copy to all sorts of filtering and transcoding possibilities.

## See it action

vgtmpeg is the underlying transcoding engine in all the native transcoding cloud apps available at [godromo.com](http://godromo.com/gmt)

## Download precompiled binaries 

We have precompiled binaries for multiple platforms ( Windows 32/64, MacOS and Linux ) at [vgtmpeg home page](http://godromo.com/gmt/vgtmpeg)

## Authors

  * Alberto Vigata [@godromo](http://twitter.com/godromo) - lead developer

Special thanks to all those who have contributed to ffmpeg over the years making it such a transforming and indispensable library for multimedia projects 


## License

vgtmpeg is available under the terms of the GNU General Public License, Version 2. Please note that
under the GPL, there is absolutely no warranty of any kind, to the extent permitted by the law.
