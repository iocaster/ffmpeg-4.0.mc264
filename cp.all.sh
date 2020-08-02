###
# armeabi-v7a
######
cd android/armeabi-v7a
cp -a include /media/sf_shared/ffmpeg.mc264/armeabi-v7a

cd lib
cp lib*-*.so /media/sf_shared/ffmpeg.mc264/armeabi-v7a/lib
cd ../../..
sync


###
# arm64-v8a
######
cd android/arm64-v8a
cp -a include /media/sf_shared/ffmpeg.mc264/arm64-v8a

cd lib
cp lib*-*.so /media/sf_shared/ffmpeg.mc264/arm64-v8a/lib
cd ../../..
sync


###
# x86
######
cd android/x86
cp -a include /media/sf_shared/ffmpeg.mc264/x86

cd lib
cp lib*-*.so /media/sf_shared/ffmpeg.mc264/x86/lib
cd ../../..
sync


###
# x86_64
######
cd android/x86_64
cp -a include /media/sf_shared/ffmpeg.mc264/x86_64

cd lib
cp lib*-*.so /media/sf_shared/ffmpeg.mc264/x86_64/lib
cd ../../..
sync

