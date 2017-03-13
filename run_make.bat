@setlocal

cd _build_make\
make
ExtendedKalmanFilter.exe ../data/sample-laser-radar-measurement-data-2.txt ../data/out-2.txt 

@endlocal