# [CVPR2023] **P**rogessive Spatio-temporal Alignment for Efficient **E**vent-based **M**otion **E**stimation (PEME)

Xueyan Huang, Yueyi Zhang*, Zhiwei Xiong

*Corresponding Author

University of Science and Technology of China (USTC), Hefei, China


## How to run
1. clone and build: 

    git clone https://github.com/huangxueyan/PEME.git
    catkin_make 

2. edit config file 
edit src/rotation_estimator/config/config.yaml when you want to test our model with different datasets.
3. roslaunch src/rotation_estimator/launch/estimation.launch

## Evaluation
We evaluate our model using the same metrics as used in the https://github.com/pbideau/Event-ST-PPP repository

    python src/eval.py -p /home/your_output_dir -e results.txt 

## Citation
If our work inspires your research or some part of the codes are useful for your work, please cite our paper:
```
@inproceedings{huang2023progressive,
  title={Progressive Spatio-Temporal Alignment for Efficient Event-Based Motion Estimation},
  author={Huang, Xueyan and Zhang, Yueyi and Xiong, Zhiwei},
  booktitle={Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition},
  pages={1537--1546},
  year={2023}
}
```

## Acknowledgement
This code is built upon the codebase from the original repository at https://github.com/Haram-kim/Globally_Aligned_Events. We would like to express our sincere gratitude to the original author, Haram Kim, for their contributions to the field and for making their code publicly available. We greatly appreciate their efforts in advancing research in this area. 
Our code is released under the MIT License.

## Contact
If you have any questions or opinions, feel free to raise them by creating an 'issue' in this repository, or contact us via hxy2020@mail.ustc.edu.cn
