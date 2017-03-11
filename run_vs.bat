@setlocal

cd _build_vs
cmake --build .

cd Debug\
ExtendedKF.exe ../../data/sample-laser-radar-measurement-data-1.txt ../../data/out-1-vs.txt 

@endlocal