import 'dart:ffi' as ffi;
import 'dart:io';
import 'dart:typed_data';
import 'package:ffi/ffi.dart';

// C++ TelemetryOutput 구조체 1:1 매핑
final class TelemetryOutput extends ffi.Struct {
  @ffi.Int32()
  external int statusCode;

  @ffi.Double()
  external double instantDraftM;

  @ffi.Float()
  external double waterlineU;

  @ffi.Float()
  external double waterlineV;

  @ffi.Int32()
  external int activeMarkCount;

  @ffi.Array(16)
  external ffi.Array<ffi.Float> markU;

  @ffi.Array(16)
  external ffi.Array<ffi.Float> markV;

  @ffi.Array(16)
  external ffi.Array<ffi.Float> markZ;

  @ffi.Int32()
  external int spinePtsCount;

  @ffi.Array(50)
  external ffi.Array<ffi.Float> spineU;

  @ffi.Array(50)
  external ffi.Array<ffi.Float> spineV;

  @ffi.Float()
  external double camRollDeg;

  @ffi.Float()
  external double camPitchDeg;

  @ffi.Int32()
  external int collectedFrames;
}

// C++ FinalSurveyResult 구조체 1:1 매핑
final class FinalSurveyResult extends ffi.Struct {
  @ffi.Int32()
  external int isValid;

  @ffi.Double()
  external double trueDraftM;

  @ffi.Double()
  external double apparentMeanM;

  @ffi.Double()
  external double simpleMeanM;

  @ffi.Double()
  external double sainflouDeductMm;

  @ffi.Double()
  external double swellHeightM;

  @ffi.Double()
  external double swellPeriodS;

  @ffi.Double()
  external double observedCycles;

  @ffi.Double()
  external double troughRmseMm;

  @ffi.Double()
  external double splitHalfDiffMm;

  @ffi.Int32()
  external int reliabilityPass;
}

// C 함수 시그니처 정의
typedef _CreateFuncC = ffi.Pointer<ffi.Void> Function(ffi.Double, ffi.Double);
typedef _CreateFuncDart = ffi.Pointer<ffi.Void> Function(double, double);

typedef _DestroyFuncC = ffi.Void Function(ffi.Pointer<ffi.Void>);
typedef _DestroyFuncDart = void Function(ffi.Pointer<ffi.Void>);

typedef _BootstrapFuncC = ffi.Int32 Function(
  ffi.Pointer<ffi.Void>,
  ffi.Pointer<ffi.Uint8>,
  ffi.Int32,
  ffi.Int32,
  ffi.Int32,
  ffi.Int32,
  ffi.Pointer<ffi.Float>,
  ffi.Pointer<ffi.Float>,
  ffi.Pointer<ffi.Float>,
);
typedef _BootstrapFuncDart = int Function(
  ffi.Pointer<ffi.Void>,
  ffi.Pointer<ffi.Uint8>,
  int,
  int,
  int,
  int,
  ffi.Pointer<ffi.Float>,
  ffi.Pointer<ffi.Float>,
  ffi.Pointer<ffi.Float>,
);

typedef _ProcessFrameC = ffi.Void Function(
  ffi.Pointer<ffi.Void>,
  ffi.Pointer<ffi.Uint8>,
  ffi.Int32,
  ffi.Int32,
  ffi.Int32,
  ffi.Float,
  ffi.Float,
  ffi.Float,
  ffi.Pointer<TelemetryOutput>,
);
typedef _ProcessFrameDart = void Function(
  ffi.Pointer<ffi.Void>,
  ffi.Pointer<ffi.Uint8>,
  int,
  int,
  int,
  double,
  double,
  double,
  ffi.Pointer<TelemetryOutput>,
);

typedef _SolveFinalC = ffi.Void Function(
  ffi.Pointer<ffi.Void>,
  ffi.Pointer<FinalSurveyResult>,
);
typedef _SolveFinalDart = void Function(
  ffi.Pointer<ffi.Void>,
  ffi.Pointer<FinalSurveyResult>,
);

class NativeDraftBridge {
  late final ffi.DynamicLibrary _lib;
  late final _CreateFuncDart _create;
  late final _DestroyFuncDart _destroy;
  late final _BootstrapFuncDart _bootstrap;
  late final _ProcessFrameDart _processFrame;
  late final _SolveFinalDart _solveFinal;

  ffi.Pointer<ffi.Void> _engineHandle = ffi.nullptr;
  ffi.Pointer< uint8_t > _frameBuffer = ffi.nullptr;
  int _allocatedSize = 0;

  final ffi.Pointer<TelemetryOutput> telemetryPtr = calloc<TelemetryOutput>();
  final ffi.Pointer<FinalSurveyResult> resultPtr = calloc<FinalSurveyResult>();

  NativeDraftBridge({double fps = 30.0, double flareDeg = 32.0}) {
    _lib = Platform.isAndroid
        ? ffi.DynamicLibrary.open('libdraft_core.so')
        : ffi.DynamicLibrary.process();

    _create = _lib.lookupFunction<_CreateFuncC, _CreateFuncDart>('draft_engine_create');
    _destroy = _lib.lookupFunction<_DestroyFuncC, _DestroyFuncDart>('draft_engine_destroy');
    _bootstrap = _lib.lookupFunction<_BootstrapFuncC, _BootstrapFuncDart>('draft_engine_bootstrap_seeds');
    _processFrame = _lib.lookupFunction<_ProcessFrameC, _ProcessFrameDart>('draft_engine_process_frame');
    _solveFinal = _lib.lookupFunction<_SolveFinalC, _SolveFinalDart>('draft_engine_solve_final');

    _engineHandle = _create(fps, flareDeg);
  }

  void _ensureFrameBuffer(Uint8List yBytes) {
    if (_allocatedSize < yBytes.length) {
      if (_frameBuffer != ffi.nullptr) calloc.free(_frameBuffer);
      _frameBuffer = calloc<ffi.Uint8>(yBytes.length);
      _allocatedSize = yBytes.length;
    }
    _frameBuffer.asTypedList(yBytes.length).setAll(0, yBytes);
  }

  bool bootstrapSeeds(
    Uint8List yBytes,
    int width,
    int height,
    int stride,
    List<double> uList,
    List<double> vList,
    List<double> zList,
  ) {
    _ensureFrameBuffer(yBytes);
    final n = uList.length;
    final uPtr = calloc<ffi.Float>(n);
    final vPtr = calloc<ffi.Float>(n);
    final zPtr = calloc<ffi.Float>(n);

    for (int i = 0; i < n; i++) {
      uPtr[i] = uList[i];
      vPtr[i] = vList[i];
      zPtr[i] = zList[i];
    }

    final ok = _bootstrap(_engineHandle, _frameBuffer, width, height, stride, n, uPtr, vPtr, zPtr);

    calloc.free(uPtr);
    calloc.free(vPtr);
    calloc.free(zPtr);
    return ok == 1;
  }

  TelemetryOutput processFrame(
    Uint8List yBytes,
    int width,
    int height,
    int stride,
    double gx,
    double gy,
    double gz,
  ) {
    _ensureFrameBuffer(yBytes);
    _processFrame(_engineHandle, _frameBuffer, width, height, stride, gx, gy, gz, telemetryPtr);
    return telemetryPtr.ref;
  }

  FinalSurveyResult solveFinalSurvey() {
    _solveFinal(_engineHandle, resultPtr);
    return resultPtr.ref;
  }

  void dispose() {
    if (_engineHandle != ffi.nullptr) _destroy(_engineHandle);
    if (_frameBuffer != ffi.nullptr) calloc.free(_frameBuffer);
    calloc.free(telemetryPtr);
    calloc.free(resultPtr);
  }
}
