#include <NvInfer.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>

#include <cstdio>
#include <cstdint>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr std::int64_t kInputElements = 1LL * 3 * 336 * 336;
constexpr std::int64_t kDescriptorElements = 384;

class Logger final : public nvinfer1::ILogger {
 public:
  void log(Severity severity, const char* message) noexcept override {
    if (severity <= Severity::kERROR && message != nullptr) {
      std::fprintf(stderr, "lingtu_dino_trt: %s\n", message);
    }
  }
};

template <typename T>
struct TrtDestroy {
  void operator()(T* value) const noexcept {
    if (value != nullptr) {
      value->destroy();
    }
  }
};

std::int64_t Volume(const nvinfer1::Dims& dims) {
  std::int64_t volume = 1;
  for (int index = 0; index < dims.nbDims; ++index) {
    if (dims.d[index] <= 0) {
      return -1;
    }
    volume *= dims.d[index];
  }
  return volume;
}

bool IsInputShape(const nvinfer1::Dims& dims) {
  return dims.nbDims == 4 && dims.d[0] == 1 && dims.d[1] == 3 && dims.d[2] == 336 &&
         dims.d[3] == 336;
}

bool IsDescriptorShape(const nvinfer1::Dims& dims) {
  return dims.nbDims == 2 && dims.d[0] == 1 && dims.d[1] == kDescriptorElements;
}

std::size_t ElementSize(nvinfer1::DataType type) {
  switch (type) {
    case nvinfer1::DataType::kFLOAT:
      return sizeof(float);
    case nvinfer1::DataType::kHALF:
      return sizeof(__half);
    case nvinfer1::DataType::kINT8:
      return sizeof(std::int8_t);
    case nvinfer1::DataType::kINT32:
      return sizeof(std::int32_t);
    case nvinfer1::DataType::kBOOL:
      return sizeof(bool);
    default:
      return 0;
  }
}

class DinoEngine {
 public:
  explicit DinoEngine(const char* engine_path) {
    std::ifstream engine_file(engine_path, std::ios::binary | std::ios::ate);
    if (!engine_file) {
      throw std::runtime_error("cannot open TensorRT engine");
    }
    const auto size = engine_file.tellg();
    if (size <= 0) {
      throw std::runtime_error("empty TensorRT engine");
    }
    engine_file.seekg(0, std::ios::beg);
    std::vector<char> bytes(static_cast<std::size_t>(size));
    if (!engine_file.read(bytes.data(), static_cast<std::streamsize>(size))) {
      throw std::runtime_error("cannot read TensorRT engine");
    }

    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (!runtime_) {
      throw std::runtime_error("cannot create TensorRT runtime");
    }
    engine_.reset(runtime_->deserializeCudaEngine(bytes.data(), bytes.size()));
    if (!engine_) {
      throw std::runtime_error("cannot deserialize TensorRT engine");
    }
    context_.reset(engine_->createExecutionContext());
    if (!context_) {
      throw std::runtime_error("cannot create TensorRT execution context");
    }

    const int binding_count = engine_->getNbBindings();
    int named_descriptor = -1;
    int shape_descriptor = -1;
    bool descriptor_shape_ambiguous = false;
    for (int binding = 0; binding < binding_count; ++binding) {
      const auto dims = engine_->getBindingDimensions(binding);
      const auto volume = Volume(dims);
      const auto element_size = ElementSize(engine_->getBindingDataType(binding));
      if (volume <= 0 || element_size == 0) {
        throw std::runtime_error("DINO engine requires fixed supported bindings");
      }
      if (engine_->bindingIsInput(binding)) {
        if (input_index_ != -1 || !IsInputShape(dims) || volume != kInputElements ||
            engine_->getBindingDataType(binding) != nvinfer1::DataType::kFLOAT) {
          throw std::runtime_error("unexpected DINO TensorRT input contract");
        }
        input_index_ = binding;
        continue;
      }
      if (IsDescriptorShape(dims) && volume == kDescriptorElements) {
        const std::string name = engine_->getBindingName(binding);
        if (name == "descriptor") {
          named_descriptor = binding;
        }
        if (shape_descriptor == -1) {
          shape_descriptor = binding;
        } else {
          descriptor_shape_ambiguous = true;
        }
      }
    }
    const int descriptor_candidate =
        named_descriptor >= 0
            ? named_descriptor
            : (descriptor_shape_ambiguous ? -1 : shape_descriptor);
    if (input_index_ < 0 || descriptor_candidate < 0) {
      throw std::runtime_error("DINO TensorRT bindings do not expose one descriptor");
    }
    descriptor_index_ = descriptor_candidate;
    descriptor_type_ = engine_->getBindingDataType(descriptor_index_);
    if (descriptor_type_ != nvinfer1::DataType::kFLOAT &&
        descriptor_type_ != nvinfer1::DataType::kHALF) {
      throw std::runtime_error("unsupported DINO descriptor type");
    }

    cudaStream_t cuda_stream = nullptr;
    if (cudaStreamCreate(&cuda_stream) != cudaSuccess) {
      throw std::runtime_error("cannot create DINO CUDA stream");
    }
    std::vector<void*> bindings(static_cast<std::size_t>(binding_count), nullptr);
    for (int binding = 0; binding < binding_count; ++binding) {
      const auto volume = Volume(engine_->getBindingDimensions(binding));
      const auto bytes = static_cast<std::size_t>(volume) *
                         ElementSize(engine_->getBindingDataType(binding));
      if (cudaMalloc(&bindings[binding], bytes) == cudaSuccess) {
        continue;
      }
      for (void* allocated : bindings) {
        if (allocated != nullptr) {
          cudaFree(allocated);
        }
      }
      cudaStreamDestroy(cuda_stream);
      throw std::runtime_error("cannot allocate DINO TensorRT binding");
    }
    bindings_ = std::move(bindings);
    stream_ = cuda_stream;
  }

  ~DinoEngine() {
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
    }
    for (void* binding : bindings_) {
      if (binding != nullptr) {
        cudaFree(binding);
      }
    }
  }

  int Encode(const float* pixels, float* descriptor) noexcept {
    if (pixels == nullptr || descriptor == nullptr) {
      return 1;
    }
    const auto input_bytes = static_cast<std::size_t>(kInputElements) * sizeof(float);
    if (cudaMemcpyAsync(bindings_[input_index_], pixels, input_bytes, cudaMemcpyHostToDevice,
                        stream_) != cudaSuccess) {
      return 2;
    }
    if (!context_->enqueueV2(bindings_.data(), stream_, nullptr)) {
      return 3;
    }
    if (descriptor_type_ == nvinfer1::DataType::kFLOAT) {
      if (cudaMemcpyAsync(descriptor, bindings_[descriptor_index_],
                          static_cast<std::size_t>(kDescriptorElements) * sizeof(float),
                          cudaMemcpyDeviceToHost, stream_) != cudaSuccess) {
        return 4;
      }
      return cudaStreamSynchronize(stream_) == cudaSuccess ? 0 : 5;
    }

    std::vector<__half> half_descriptor(static_cast<std::size_t>(kDescriptorElements));
    if (cudaMemcpyAsync(half_descriptor.data(), bindings_[descriptor_index_],
                        half_descriptor.size() * sizeof(__half), cudaMemcpyDeviceToHost,
                        stream_) != cudaSuccess ||
        cudaStreamSynchronize(stream_) != cudaSuccess) {
      return 4;
    }
    for (std::size_t index = 0; index < half_descriptor.size(); ++index) {
      descriptor[index] = __half2float(half_descriptor[index]);
    }
    return 0;
  }

 private:
  Logger logger_;
  std::unique_ptr<nvinfer1::IRuntime, TrtDestroy<nvinfer1::IRuntime>> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine, TrtDestroy<nvinfer1::ICudaEngine>> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext, TrtDestroy<nvinfer1::IExecutionContext>> context_;
  std::vector<void*> bindings_;
  int input_index_ = -1;
  int descriptor_index_ = -1;
  nvinfer1::DataType descriptor_type_ = nvinfer1::DataType::kFLOAT;
  cudaStream_t stream_ = nullptr;
};

}  // namespace

extern "C" void* dino_create(const char* engine_path) noexcept {
  if (engine_path == nullptr) {
    return nullptr;
  }
  try {
    return new DinoEngine(engine_path);
  } catch (const std::exception& error) {
    std::fprintf(stderr, "lingtu_dino_trt: %s\n", error.what());
    return nullptr;
  } catch (...) {
    return nullptr;
  }
}

extern "C" int dino_encode(void* handle, const float* pixels, float* descriptor) noexcept {
  if (handle == nullptr) {
    return 1;
  }
  return static_cast<DinoEngine*>(handle)->Encode(pixels, descriptor);
}

extern "C" void dino_destroy(void* handle) noexcept {
  delete static_cast<DinoEngine*>(handle);
}
