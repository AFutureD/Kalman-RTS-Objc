# Kalman-RTS-Objc

## Usage


```

int main(int argc, char *argv[]) {
    @autoreleasepool {
        KalmanFilter * kf = [KalmanFilter new];
        NSArray * arr = @[
            @-53, @-54, @-55, @-59, @-59, @-60, @-52, @-44, @-44, @-49, @-45, @-46, @-46, @-49, @-47, @-50, @-50, @-50, @-46, @-43, @-43, @-44, @-47, @-46, @-46, @-46, @-46, @-46, @-44, @-42, @-46, @-46, @-46, @-43, @-46
        ];
        
        [kf setupRTSCount:arr.count];
        
        NSArray * kamlam = [kf estimateArray:arr];
        
        NSArray * rts = [kf smoothArrayWithRTS];
        
        NSLog(@"kalman: %@ rts: %@", kamlam, rts);
    }
}


```

## 过程

### Python 3 的实现：

<img width="775" alt="image_6490965230939171" src="https://user-images.githubusercontent.com/17228538/191203023-7640d8af-907b-4214-a1fd-d4ea093f29f4.png">

RTS 的结果是蓝色线条，对比设备提供的测量结果（绿线），我们认为这是一个理想的结果。

### OC 实现

首先是 RTS 的算法实现：

<img width="1000" alt="image_0329179758197623" src="https://user-images.githubusercontent.com/17228538/191203376-86aa75d6-db36-44cc-b8e6-ec95873f5291.png">

考虑现状，我们只需要实现 一维的 RTS，于是我们讲 A、H 等参数默认为 1（上图Python 的实现也是 1 ）。

以下是我的实现：

<img width="1795" alt="image_06116335727788269" src="https://user-images.githubusercontent.com/17228538/191203568-b6ad63bf-aa50-431f-874b-0ce7ca40da7b.png">
其中蓝色线条是 最终的实现。橙色线条是，利用 10^((baseRSSI-currentRSSI)/(10 * 2)) 计算得到的距离曲线用来参考。灰色线条是原始值。


放大最后一部分，我认为是一个比较理想的实现。

<img width="772" alt="image_20047064422098015" src="https://user-images.githubusercontent.com/17228538/191203822-91654f15-8476-4e0c-a919-23b9a64b70c9.png">

