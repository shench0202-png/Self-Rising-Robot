#ifndef NEURAL_NETWORK_H
#define NEURAL_NETWORK_H

#include <Arduino.h>

// Fixed policy shape used by Arduino/robo03/policy_network.h:
// observation[4] -> tanh(64) -> tanh(64) -> action[2].
static const uint8_t NN_INPUT_DIM = 4;
static const uint8_t NN_HIDDEN0_DIM = 64;
static const uint8_t NN_HIDDEN1_DIM = 64;
static const uint8_t NN_OUTPUT_DIM = 2;

struct NeuralNetworkWeights {
  const float *w0;  // [NN_HIDDEN0_DIM][NN_INPUT_DIM], row-major
  const float *b0;  // [NN_HIDDEN0_DIM]
  const float *w1;  // [NN_HIDDEN1_DIM][NN_HIDDEN0_DIM], row-major
  const float *b1;  // [NN_HIDDEN1_DIM]
  const float *w2;  // [NN_OUTPUT_DIM][NN_HIDDEN1_DIM], row-major
  const float *b2;  // [NN_OUTPUT_DIM]
};

class NeuralNetwork {
public:
  NeuralNetwork();
  explicit NeuralNetwork(const NeuralNetworkWeights &weights);

  void setWeights(const NeuralNetworkWeights &weights);
  bool isConfigured() const;

  // Computes action = policy(observation). Returns false and clears output if
  // any weight pointer is missing.
  bool forward(const float observation[NN_INPUT_DIM],
               float action[NN_OUTPUT_DIM]);

  static float clamp(float value, float lower, float upper);

private:
  NeuralNetworkWeights weights_;
  float hidden0_[NN_HIDDEN0_DIM];
  float hidden1_[NN_HIDDEN1_DIM];

  static float activation(float value);
  static float denseNeuron(const float *weights,
                           const float *bias,
                           uint8_t row,
                           uint8_t input_dim,
                           const float *input);

  void clearOutput(float action[NN_OUTPUT_DIM]) const;
};

#endif
