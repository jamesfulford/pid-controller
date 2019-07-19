# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

![Car staying on the road](./cover.png)

The goal of this project is to keep the car on the road with a steering wheel. The catch is that the only (useful) sensor measurement available is Cross Track Error (distance from car to center of the road). By weighting the Present deviation, the Derivative of the deviation (no crazy swings), and the Integral of the deviation (cancel out any inherent bias built into car), I got the car to stay on the road.

## Reflection

The tricky part of this project is not implementing a PID controller - I completed that almost immediately, with some improvements later. The hard part is finding the right weights, which in this context are called *gains*, to apply to the P, I, and D components to determine how much to steer. I implemented backpropagation to incrementally improve the gains while driving.

This PID implementation differs slightly from the typical PID controller in that the Integral component does not consider all previous cross track errors, but only the most recent 35. This way, the I component does not grow without bounds. This makes the gain for I have a stable meaning. (a low fixed `I` gain may not influence steering early, but give it a couple laps and the accumulated error could hijack the linear combination)

I spent a week manually tuning the gains. If they are too low, it will not steer enough. If they are too high, it will steer too much and fall off the road. If unbalanced, it will swing from side to side, continually overcorrecting for the previous wild swing until it falls off the road. I got stuck in a local minima. After a failed backpropagation attempt (confused deviation from center of road with error and accidentally penalized gains for being positive), I eventually came to a proper backpropagation solution, identified the local minima, used vanilla gains provided in class, and let backpropagation do its magic. Very shortly after, the car got around the track. I let it go on for about an hour longer and then extracted the gains.

In short, backpropagation penalizes each component based on how much error it was introducing to the linear combination and how much error there was in total (root mean squared error). For the I component, since individual errors can cancel out if added directly (left is negative, right is positive, but large values in either direction are still very error-some), the error contribution considered for backprop has to be the sum of the absolute values of these I-based error contributions.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.
