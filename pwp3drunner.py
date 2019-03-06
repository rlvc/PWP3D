import os

program_path = "./cmake-build-release/Application/PWP3DAPP "
mask_path = " ./Files/Masks/RES/temp_mask%04d.png "
pose_path = " ./Files/Masks/RES/pose%04d.txt "

print('*'*10)
for i in range(0, 3):
    main = program_path + mask_path + pose_path + str(i)
    print(main)
    r_v = os.system(main)
    print(r_v)
    print('*' * 10)

