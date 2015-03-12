# About

`vgtmpeg` is a ffmpeg drop-in replacement that adds a number of additional features to the stock ffmpeg and libavformat/libavcodec libraries:

* ***DVD reading*** capability through the addition of a new `dvdurl` protocol using [libdvdread-vgtmpeg](http://github.com/concalma/libdvdread-vgtmpeg) 
* ***Bluray reading*** capability through the use of libbluray
* ***Rich metadata*** availability of DVD/Bluray information into transcoded streams: chapters,language info,subtitles are all passed on to the transcoded content if the output supports it
* ***pipe control and reporting*** `vgtmpeg` ands a control interface to ffmpeg to start, stop, transcode as well as retrieve extra progress information. For example `vgtmpeg` can output thumbnails of the ongoing transcode through the pipe. This offers simple richer integration control of `vgtmpeg` into other applications
* ***multiplatform releases*** Releases of vgtmpeg include precompiled binaries for major platforms Windows, OS X, Linux both in 32 and 64bit with a vast array of built-in formats and codecs [Download them here](http://godromo.com/gmt/vgtmpeg)



# Download binaries 

Precompiled binaries for multiple platforms ( Windows 32/64, MacOS and Linux ) at [vgtmpeg home page](http://godromo.com/gmt/vgtmpeg)
### See it action

vgtmpeg is the underlying transcoding engine in all the native transcoding cloud apps available at [godromo.com](http://godromo.com/gmt)

# Author

  * Alberto Vigata [learn more](http://vigata.com/about)


--------

## Compiling
`vgtmpeg` uses most of the standard libraries like libx264 and libx265 as ffmpeg does, so building the source tree is mostly similar to that of ffmpeg.


For DVD support though, `vgtmpeg` uses [libdvdread-vgtmpeg](http://github.com/concalma/libdvdread-vgtmpeg) and it must be installed in your build system.

## DVD/Bluray support
vgtmpeg adds support for DVDs and BD in its version of libavformat. DVD/BD support is implemented by adding a new ‘dvdurl’ protocol that can parse DVD folders, DVD ISO files, DVD devices and more. The ‘bdurl’ protocol can parse bluray folders. All the regular features available in vgtmpeg/ffmpeg are still available when a dvd url or a bd url is used. From direct stream copy to all sorts of filtering and transcoding possibilities.

Using DVDs with vgtmpeg
Strictly one can open a DVD folder, ISO file.. by using a DVD url like this:

``` vgtmpeg -i dvd://path_to_dvd  outfile```

When using the above format vgtmpeg will inspect the ‘path_to_dvd’ location looking for a DVD image in the form of a ISO file, or a DVD folder. ‘path_to_dvd’ can also be any of the individual files inside the VIDEO_TS folder, ‘vgtmpeg’ will figure out the rest.

By default, the title with the longest duration is opened when using the above syntax. If you want to rely on this behavior, the use of the dvd:// is not required and just specifying the path will suffice. One can however, ask for specific titles to be used as the input using a url query var:

``` vgtmpeg -i dvd://path_to_dvd?title=5 outfile```
This will open the title 5 (if available) of the DVD. If you want to know what is available on a DVD simply type:

``` vgtmpeg -i dvd://path_to_dvd```
Using Bluray folders with vgtmpeg
Strictly one can open a Bluray folder,by using a BD url like this:

``` vgtmpeg -i bd://path_to_bd  outfile```
When using the above format vgtmpeg will inspect the ‘path_to_bd’ location looking for a Bluray folder image. The folder will be inspected for a bluray like structure and analyzed looking for titles and video and audio streams.

By default, the title with the longest duration is opened when using the above syntax. If you want to rely on this behavior, the use of the bd:// is not required and just specifying the path will suffice. One can however, ask for specific titles to be used as the input using a url query var:

``` vgtmpeg -i bd://path_to_bd?title=5 outfile```
This will open the title 5 (if available) of the BD. If you want to know what is available on a BD simply type:

``` vgtmpeg -i bd://path_to_bd```
###DVD and Bluray paths
The path to use for the -i option is flexible. You can point to an IFO file, a VIDEO_TS folder, the root of a VIDEO_TS folder or an ISO file containing a VIDEO_TS folder. In any of the cases, vgtmpeg will try to figure out the root file of the DVD from this information and if successful will open the DVD and load the information in the IFO files.

At the moment only Bluray folders are supported and you should point to the root of the Bluray folder.





# License

vgtmpeg is available under the terms of the GNU General Public License, Version 2. Please note that
under the GPL, there is absolutely no warranty of any kind, to the extent permitted by the law.

