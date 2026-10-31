#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <cstdio>
#include <cstdint>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <vector>

namespace {

constexpr std::int64_t kInputElements = 1LL * 3 * 640 * 640;
constexpr std::int64_t kOutputElements = 1LL * 5 * 8400;

class Logger final : public nvinfer1::ILogger {
 public:
  void log(Severity severity, const char* message) noexcept override {
    if (severity <= Severity::kERROR && message != nullptr) {
      std::fprintf(stderr, "lingtu_yolo_trt: %s\n", message);
    }
  }
};

template <typename T>
struct TrtDestroy {
  void operator()(T* value) const noexcept {
    if (value != nullptr) value->destroy();
  }
};

std::int64_t Volume(const nvinfer1::Dims& dims) {
  std::int64_t volume = 1;
  for (int index = 0; index < dims.nbDims; ++index) {
    if (dims.d[index] <= 0) return -1;
    volume *= dims.d[index];
  }
  return volume;
}

bool IsInput(const nvinfer1::Dims& dims) {
  return dims.nbDims == 4 && dims.d[0] == 1 && dims.d[1] == 3 && dims.d[2] == 640 &&
         dims.d[3] == 640;
}

bool IsOutput(const nvinfer1::Dims& dims) {
  return dims.nbDims == 3 && dims.d[0] == 1 && dims.d[1] == 5 && dims.d[2] == 8400;
}

class YoloEngine {
 public:
  explicit YoloEngine(const char* engine_path) {
    std::ifstream stream(engine_path, std::ios::binary | std::ios::ate);
    if (!stream) throw std::runtime_error("cannot open TensorRT engine");
    const auto size = stream.tellg();
    if (size <= 0) throw std::runtime_error("empty TensorRT engine");
    stream.seekg(0, std::ios::beg);
    std::vector<char> bytes(static_cast<std::size_t>(size));
    if (!stream.read(bytes.data(), static_cast<std::streamsize>(size))) {
      throw std::runtime_error("cannot read TensorRT engine");
    }

    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (!runtime_) throw std::runtime_error("cannot create TensorRT runtime");
    engine_.reset(runtime_->deserializeCudaEngine(bytes.data(), bytes.size()));
    if (!engine_) throw std::runtime_error("cannot deserialize TensorRT engine");
    context_.reset(engine_->createExecutionContext());
    if (!context_) throw std::runtime_error("cannot create TensorRT execution context");

    const int count = engine_->getNbBindings();
    bindings_.assign(static_cast<std::size_t>(count), nullptr);
    for (int binding = 0; binding < count; ++binding) {
      const auto dims = engine_->getBindingDimensions(binding);
      const auto type = engine_->getBindingDataType(binding);
      const auto volume = Volume(dims);
      if (type != nvinfer1::DataType::kFLOAT || volume <= 0) {
        throw std::runtime_error("YOLO engine requires fixed float32 bindings");
      }
      if (engine_->bindingIsInput(binding)) {
        if (input_index_ != -1 || !IsInput(dims) || volume != kInputElements) {
          throw std::runtime_error("unexpected YOLO TensorRT input contract");
        }
        input_index_ = binding;
      } else {
        if (output_index_ != -1 || !IsOutput(dims) || volume != kOutputElements) {
          throw std::runtime_error("unexpected YOLO TensorRT output contract");
        }
        output_index_ = binding;
      }
    }
    if (input_index_ < 0 || output_index_ < 0) {
      throw std::runtime_error("YOLO TensorRT bindings are incomplete");
    }
    if (cudaStreamCreate(&stream_) != cudaSuccess) {
      throw std::runtime_error("cannot create YOLO CUDA stream");
    }
    for (int binding = 0; binding < count; ++binding) {
      const auto bytes_count = static_cast<std::size_t>(
                                   Volume(engine_->getBindingDimensions(binding))) *
                               sizeof(float);
      if (cudaMalloc(&bindings_[binding], bytes_count) != cudaSuccess) {
        for (void* allocated : bindings_) {
          if (allocated != nullptr) cudaFree(allocated);
        }
        bindings_.clear();
        cudaStreamDestroy(stream_);
        stream_ = nullptr;
        throw std::runtime_error("cannot allocate YOLO TensorRT binding");
      }
    }
  }

  ~YoloEngine() {
    for (void* binding : bindings_) {
      if (binding != nullptr) cudaFree(binding);
    }
    if (stream_ != nullptr) cudaStreamDestroy(stream_);
  }

  int Infer(const float* pixels, float* output) noexcept {
    if (pixels == nullptr || output == nullptr) return 1;
    if (cudaMemcpyAsync(bindings_[input_index_], pixels,
                        static_cast<std::size_t>(kInputElements) * sizeof(float),
                        cudaMemcpyHostToDevice, stream_) != cudaSuccess) {
      return 2;
    }
    if (!context_->enqueueV2(bindings_.data(), stream_, nullptr)) return 3;
    if (cudaMemcpyAsync(output, bindings_[output_index_],
                        static_cast<std::size_t>(kOutputElements) * sizeof(float),
                        cudaMemcpyDeviceToHost, stream_) != cudaSuccess) {
      return 4;
    }
    return cudaStreamSynchronize(stream_) == cudaSuccess ? 0 : 5;
  }

 private:
  Logger logger_;
  std::unique_ptr<nvinfer1::IRuntime, TrtDestroy<nvinfer1::IRuntime>> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine, TrtDestroy<nvinfer1::ICudaEngine>> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext, TrtDestroy<nvinfer1::IExecutionContext>> context_;
  std::vector<void*> bindings_;
  int input_index_ = -1;
  int output_index_ = -1;
  cudaStream_t stream_ = nullptr;
};

}  // namespace

extern "C" void* yolo_create(const char* engine_path) noexcept {
  if (engine_path == nullptr) return nullptr;
  try {
    return new YoloEngine(engine_path);
  } catch (const std::exception& error) {
    std::fprintf(stderr, "lingtu_yolo_trt: %s\n", error.what());
    return nullptr;
  } catch (...) {
    return nullptr;
  }
}

extern "C" int yolo_infer(void* handle, const float* pixels, float* output) noexcept {
  if (handle == nullptr) return 1;
  return static_cast<YoloEngine*>(handle)->Infer(pixels, output);
}

extern "C" void yolo_destroy(void* handle) noexcept {
  delete static_cast<YoloEngine*>(handle);
}
