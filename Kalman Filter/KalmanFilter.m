#import <Foundation/Foundation.h>
#import "KalmanFilter.h"

@interface KalmanFilter()

@property (nonatomic) float est_last;
@property (nonatomic) float P_last;

//the noise in the system
@property (nonatomic) float Q;
@property (nonatomic) float R;

// Kalman Gain
@property (nonatomic) float K;

// coveriance
@property (nonatomic) float P;

@property (nonatomic) NSUInteger keptCount;
@property (nonatomic, strong ) NSMutableArray *kalman_mu;
@property (nonatomic, strong ) NSMutableArray *kalman_cov;
@property (nonatomic, strong ) NSMutableArray *keptData;

@end


@implementation KalmanFilter

- (instancetype)init
{
    self = [super init];
    if (self) {
        [self initParam];
    }
    return self;
}

- (void)initParam {
    _est_last = 0;
    _P_last = 0;
    
    _Q = 0.01;
    _R = 0.1;
    _keptCount = 50;
    
    _keptData = [NSMutableArray arrayWithCapacity:_keptCount];
    _kalman_mu = [NSMutableArray arrayWithCapacity:_keptCount];
    _kalman_cov = [NSMutableArray arrayWithCapacity:_keptCount];
}

#pragma mark -

- (void)setupEst:(float)est_last P:(float)P_last Q:(float )Q R:(float )R {
    _est_last = est_last;
    _P_last = P_last;
    
    _Q = Q;
    _R = R;
}

#pragma mark filter

- (float)estimateElement:(float)value {
    if (self.keptData.count >= self.keptCount) {
        [self.keptData removeObjectAtIndex:0];
        [self.kalman_mu removeObjectAtIndex:0];
        [self.kalman_cov removeObjectAtIndex:0];
    }
    if (fabs(_est_last - 0) < 0.1) {
        _est_last = value;
    }
    
    float _measured = value;
    
    float _temp_est = _est_last;
    float _P_temp = _P_last + _Q;
    
    _K = _P_temp * (1.0 / (_P_temp + _R));

    float _est = _temp_est + _K * (_measured - _temp_est);
    _P = (1 - _K) * _P_temp;
    
    _P_last = _P;
    _est_last = _est;
    
    [self.keptData addObject:@(value)];
    [self.kalman_mu addObject:@(_est_last)];
    [self.kalman_cov addObject:@(_P_last)];
    
    return _est;
}

- (NSArray *)smoothArrayWithRTS {
    
    if (self.keptData.count == 1) {
        return [self.keptData copy];
    }
    
    float _rts_P_last = 0, _rts_est_last = 0;
    
    
    for (unsigned long i = self.keptData.count - 2; i > 0; i--) {
        
        float _temp_est = [self.kalman_mu[i] doubleValue];
        float _P_temp = [self.kalman_cov[i] doubleValue] + _Q;
        
        float _G = [self.kalman_cov[i] doubleValue] / _P_temp;
        
        _rts_est_last = [self.kalman_mu[i] doubleValue]
        + _G * ([self.kalman_mu[i + 1] doubleValue] - _temp_est);
        
        _rts_P_last = [self.kalman_cov[i] doubleValue]
        + _G * _G * ([self.kalman_cov[i + 1] doubleValue] - _P_temp);
        
        [self.kalman_mu setObject:@(_rts_est_last) atIndexedSubscript:i];
        [self.kalman_cov setObject:@(_rts_P_last) atIndexedSubscript:i];
    }
    return [self.kalman_mu copy];
}

- (NSArray *)estimateArray:(NSArray *)arrData {
    
    NSMutableArray *arrResult = [NSMutableArray arrayWithCapacity:arrData.count];
    
    for (int i = 0; i < arrData.count; i++) {
        float _measured = [arrData[i] doubleValue];
        
        float temp = [self estimateElement:_measured];

        [arrResult setObject:@(temp) atIndexedSubscript:i];
    }
    
    return arrResult;
}


- (void)setupRTSCount:(NSUInteger)count {
    _keptCount = count;
}

@end

int main(int argc, char *argv[]) {
    @autoreleasepool {
        KalmanFilter * kf = [KalmanFilter new];
        NSArray * arr = @[
            @-53, @-54, @-55, @-59, @-59, @-60, @-52, @-44, @-44, @-49, @-45, @-46, @-46, @-49, @-47, @-50, @-50, @-50, @-46, @-43, @-43, @-44, @-47, @-46, @-46, @-46, @-46, @-46, @-44, @-42, @-46, @-46, @-46, @-43, @-46, @-46, @-46, @-45, @-54, @-75, @-75, @-71, @-71, @-65, @-65, @-67, @-68, @-67, @-67, @-70, @-74, @-76, @-76, @-84, @-76, @-79, @-79, @-68, @-68, @-67, @-67, @-65, @-68, @-66, @-66, @-67, @-67, @-67, @-67, @-84, @-82, @-82, @-82, @-85, @-91, @-98, @-98, @-98, @-98, @-83, @-83, @-90, @-85, @-86, @-86, @-90, @-86, @-81, @-81, @-81, @-84, @-89, @-89, @-87, @-85, @-83, @-83, @-85, @-85, @-82, @-82, @-92, @-85, @-86, @-86, @-85, @-86, @-86, @-83, @-84, @-81, @-81, @-85, @-83, @-84, @-83, @-83, @-92, @-92, @-92, @-86, @-87, @-86, @-86, @-92, @-92, @-93, @-93, @-87, @-88, @-86, @-86, @-89, @-73, @-77, @-77, @-76, @-73, @-73, @-73, @-72, @-76, @-82, @-82, @-71, @-74, @-70, @-70, @-72, @-79, @-79, @-79, @-85, @-82, @-65, @-65, @-61, @-60, @-64, @-64, @-49, @-56, @-61, @-61, @-53, @-50, @-51, @-51, @-57, @-59, @-56, @-56, @-53, @-55, @-57, @-57, @-59, @-59
        ];
        
        [kf setupRTSCount:arr.count];
        
        NSArray * kamlam = [kf estimateArray:arr];
        
        NSArray * rts = [kf smoothArrayWithRTS];
        
        NSLog(@"kalman: %@ rts: %@", kamlam, rts);
    }
}