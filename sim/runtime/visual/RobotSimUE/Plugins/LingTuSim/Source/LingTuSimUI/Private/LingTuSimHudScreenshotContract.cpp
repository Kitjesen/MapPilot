#include "LingTuSimHudScreenshotContract.h"

#include "HAL/FileManager.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "LingTuSimRuntimeUIStatus.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"

#if PLATFORM_WINDOWS
#include "Windows/AllowWindowsPlatformTypes.h"
#include <Windows.h>
#include "Windows/HideWindowsPlatformTypes.h"
#endif

#if PLATFORM_UNIX
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace LingTuSim::UI
{
    namespace
    {
        struct FTargetDefinition final
        {
            const TCHAR* CommandLineName;
            const TCHAR* ExpectedFilename;
            const TCHAR* ModeName;
            ERuntimeUIMode Mode;
        };

        constexpr FTargetDefinition TargetDefinitions[] = {
            {
                TEXT("LingTuHudDriveScreenshot"),
                TEXT("hud-drive.png"),
                TEXT("drive"),
                ERuntimeUIMode::Drive,
            },
            {
                TEXT("LingTuHudTacticalScreenshot"),
                TEXT("hud-tactical.png"),
                TEXT("tactical"),
                ERuntimeUIMode::Tactical,
            },
            {
                TEXT("LingTuHudMenuRecordingScreenshot"),
                TEXT("hud-menu-recording.png"),
                TEXT("menu_recording"),
                ERuntimeUIMode::Pause,
            },
        };

        bool DecodeAssignmentValue(
            const FString& RawValue,
            FString& OutValue)
        {
            OutValue = RawValue;
            const bool bHasLeadingQuote = OutValue.StartsWith(TEXT("\""));
            const bool bHasTrailingQuote = OutValue.EndsWith(TEXT("\""));
            if (bHasLeadingQuote != bHasTrailingQuote
                || (bHasLeadingQuote && OutValue.Len() < 2))
            {
                return false;
            }
            if (bHasLeadingQuote)
            {
                OutValue = OutValue.Mid(1, OutValue.Len() - 2);
            }
            return !OutValue.Contains(TEXT("\""));
        }

        bool AssignmentValues(
            const TArray<FString>& Switches,
            const TCHAR* Name,
            TArray<FString>& OutValues,
            FString& OutError)
        {
            OutValues.Reset();
            const FString Prefix = FString::Printf(TEXT("%s="), Name);
            for (const FString& Switch : Switches)
            {
                if (Switch.StartsWith(Prefix, ESearchCase::CaseSensitive))
                {
                    FString Value;
                    if (!DecodeAssignmentValue(Switch.Mid(Prefix.Len()), Value))
                    {
                        OutError = TEXT("hud_screenshot_argument_quotes_invalid");
                        return false;
                    }
                    OutValues.Add(MoveTemp(Value));
                }
            }
            return true;
        }

        FString CanonicalAbsolutePath(const FString& Path)
        {
            FString Result = FPaths::ConvertRelativePathToFull(Path);
            FPaths::NormalizeFilename(Result);
            FPaths::CollapseRelativeDirectories(Result);
            return Result;
        }

#if PLATFORM_WINDOWS
        FString RemoveWindowsDevicePrefix(FString Path)
        {
            if (Path.StartsWith(TEXT("\\\\?\\UNC\\"), ESearchCase::IgnoreCase))
            {
                Path = TEXT("\\\\") + Path.Mid(8);
            }
            else if (Path.StartsWith(TEXT("\\\\?\\"), ESearchCase::IgnoreCase))
            {
                Path.RightChopInline(4, EAllowShrinking::No);
            }
            return Path;
        }

        bool InspectWindowsPathWithoutFollowingReparsePoint(
            const FString& Path,
            const bool bExpectedDirectory,
            FString& OutError)
        {
            const DWORD Attributes = ::GetFileAttributesW(*Path);
            if (Attributes == INVALID_FILE_ATTRIBUTES)
            {
                OutError = TEXT("path_component_missing");
                return false;
            }
            if ((Attributes & FILE_ATTRIBUTE_REPARSE_POINT) != 0)
            {
                OutError = TEXT("reparse_path_component");
                return false;
            }
            const bool bIsDirectory = (Attributes & FILE_ATTRIBUTE_DIRECTORY) != 0;
            if (bIsDirectory != bExpectedDirectory)
            {
                OutError = bExpectedDirectory
                    ? TEXT("path_component_not_directory")
                    : TEXT("target_not_plain_file");
                return false;
            }

            const DWORD Flags = FILE_FLAG_OPEN_REPARSE_POINT
                | (bExpectedDirectory ? FILE_FLAG_BACKUP_SEMANTICS : 0);
            HANDLE Handle = ::CreateFileW(
                *Path,
                FILE_READ_ATTRIBUTES,
                FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
                nullptr,
                OPEN_EXISTING,
                Flags,
                nullptr);
            if (Handle == INVALID_HANDLE_VALUE)
            {
                OutError = TEXT("no_follow_open_failed");
                return false;
            }

            BY_HANDLE_FILE_INFORMATION Information{};
            const bool bInformationRead = ::GetFileInformationByHandle(Handle, &Information) != 0;
            if (!bInformationRead
                || (Information.dwFileAttributes & FILE_ATTRIBUTE_REPARSE_POINT) != 0
                || (((Information.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0)
                    != bExpectedDirectory))
            {
                ::CloseHandle(Handle);
                OutError = TEXT("reparse_or_type_changed_after_open");
                return false;
            }

            TArray<WCHAR> FinalPathBuffer;
            FinalPathBuffer.SetNumUninitialized(32'768);
            const DWORD FinalPathLength = ::GetFinalPathNameByHandleW(
                Handle,
                FinalPathBuffer.GetData(),
                static_cast<DWORD>(FinalPathBuffer.Num()),
                FILE_NAME_NORMALIZED | VOLUME_NAME_DOS);
            ::CloseHandle(Handle);
            if (FinalPathLength == 0
                || FinalPathLength >= static_cast<DWORD>(FinalPathBuffer.Num()))
            {
                OutError = TEXT("final_path_read_failed");
                return false;
            }
            FinalPathBuffer[static_cast<int32>(FinalPathLength)] = 0;
            const FString FinalPath = CanonicalAbsolutePath(
                RemoveWindowsDevicePrefix(FString(FinalPathBuffer.GetData())));
            if (!FPaths::IsSamePath(FinalPath, CanonicalAbsolutePath(Path)))
            {
                OutError = TEXT("final_path_mismatch");
                return false;
            }
            return true;
        }
#endif

        bool ValidateExistingPlainPath(
            const FString& Path,
            const bool bExpectedDirectory,
            FString& OutError)
        {
            IFileManager& FileManager = IFileManager::Get();
            if (FileManager.IsSymlink(*Path))
            {
                OutError = TEXT("reparse_path_component");
                return false;
            }
#if PLATFORM_WINDOWS
            return InspectWindowsPathWithoutFollowingReparsePoint(
                Path,
                bExpectedDirectory,
                OutError);
#else
            const bool bExists = bExpectedDirectory
                ? FileManager.DirectoryExists(*Path)
                : FileManager.FileExists(*Path);
            if (!bExists)
            {
                OutError = TEXT("path_component_missing");
                return false;
            }
            return true;
#endif
        }

        bool ValidatePlainAncestorChain(const FString& TargetPath, FString& OutError)
        {
            FString Current = CanonicalAbsolutePath(FPaths::GetPath(TargetPath));
            if (Current.IsEmpty())
            {
                OutError = TEXT("target_parent_missing");
                return false;
            }
            while (!Current.IsEmpty())
            {
#if PLATFORM_WINDOWS
                const bool bIsDrive = FPaths::IsDrive(Current);
                FString PathToInspect = Current;
                if (bIsDrive
                    && !PathToInspect.EndsWith(TEXT("/"))
                    && !PathToInspect.EndsWith(TEXT("\\")))
                {
                    PathToInspect += TEXT("/");
                }
#else
                const bool bIsDrive = FPaths::IsDrive(Current);
                const FString& PathToInspect = Current;
#endif
                if (!ValidateExistingPlainPath(PathToInspect, true, OutError))
                {
                    return false;
                }
                if (bIsDrive)
                {
                    break;
                }
                const FString Parent = CanonicalAbsolutePath(FPaths::GetPath(Current));
                if (Parent.IsEmpty() || FPaths::IsSamePath(Parent, Current))
                {
                    break;
                }
                Current = Parent;
            }
            return true;
        }

        bool ValidateFileState(
            const FString& Path,
            const bool bMustExist,
            const TCHAR* MissingError,
            const TCHAR* StaleError,
            FString& OutError)
        {
#if PLATFORM_WINDOWS
            HANDLE Handle = ::CreateFileW(
                *Path,
                FILE_READ_ATTRIBUTES,
                FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
                nullptr,
                OPEN_EXISTING,
                FILE_FLAG_OPEN_REPARSE_POINT | FILE_FLAG_BACKUP_SEMANTICS,
                nullptr);
            if (Handle == INVALID_HANDLE_VALUE)
            {
                const DWORD Error = ::GetLastError();
                if (Error == ERROR_FILE_NOT_FOUND || Error == ERROR_PATH_NOT_FOUND)
                {
                    if (bMustExist)
                    {
                        OutError = MissingError;
                        return false;
                    }
                    return true;
                }
                OutError = TEXT("target_state_cannot_be_verified_without_following");
                return false;
            }

            BY_HANDLE_FILE_INFORMATION Information{};
            const bool bInformationRead =
                ::GetFileInformationByHandle(Handle, &Information) != 0;
            ::CloseHandle(Handle);
            if (!bInformationRead)
            {
                OutError = TEXT("target_state_cannot_be_verified_without_following");
                return false;
            }
            if ((Information.dwFileAttributes & FILE_ATTRIBUTE_REPARSE_POINT) != 0)
            {
                OutError = TEXT("reparse_target_file");
                return false;
            }
            if ((Information.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0)
            {
                OutError = TEXT("target_is_directory");
                return false;
            }
            if (!bMustExist)
            {
                OutError = StaleError;
                return false;
            }
            return InspectWindowsPathWithoutFollowingReparsePoint(
                Path,
                false,
                OutError);
#else
            IFileManager& FileManager = IFileManager::Get();
            if (FileManager.IsSymlink(*Path))
            {
                OutError = TEXT("reparse_target_file");
                return false;
            }
            const bool bFileExists = FileManager.FileExists(*Path);
            const bool bDirectoryExists = FileManager.DirectoryExists(*Path);
            if (bDirectoryExists)
            {
                OutError = TEXT("target_is_directory");
                return false;
            }
            if (bMustExist)
            {
                if (!bFileExists)
                {
                    OutError = MissingError;
                    return false;
                }
                return ValidateExistingPlainPath(Path, false, OutError);
            }
            if (bFileExists)
            {
                OutError = StaleError;
                return false;
            }
            return true;
#endif
        }

        uint32 ReadBigEndianUint32(const uint8* Bytes)
        {
            return (static_cast<uint32>(Bytes[0]) << 24)
                | (static_cast<uint32>(Bytes[1]) << 16)
                | (static_cast<uint32>(Bytes[2]) << 8)
                | static_cast<uint32>(Bytes[3]);
        }

        void SetExactUint64(
            const TSharedRef<FJsonObject>& Object,
            const TCHAR* Field,
            const uint64 Value)
        {
            Object->SetField(
                Field,
                MakeShared<FJsonValueNumberString>(LexToString(Value)));
        }

        void SetExactInt64(
            const TSharedRef<FJsonObject>& Object,
            const TCHAR* Field,
            const int64 Value)
        {
            Object->SetField(
                Field,
                MakeShared<FJsonValueNumberString>(LexToString(Value)));
        }

        const TCHAR* AckStatusName(const EControlAckStatus Status)
        {
            switch (Status)
            {
            case EControlAckStatus::Pending:
                return TEXT("pending");
            case EControlAckStatus::Accepted:
                return TEXT("accepted");
            case EControlAckStatus::Rejected:
                return TEXT("rejected");
            case EControlAckStatus::Released:
                return TEXT("released");
            case EControlAckStatus::TimeoutZero:
                return TEXT("timeout_zero");
            case EControlAckStatus::Confirmed:
                return TEXT("confirmed");
            default:
                return TEXT("rejected");
            }
        }

        const TCHAR* BindingStateName(const EControlBindingState State)
        {
            switch (State)
            {
            case EControlBindingState::Unbound:
                return TEXT("UNBOUND");
            case EControlBindingState::Prepared:
                return TEXT("PREPARED");
            case EControlBindingState::Active:
                return TEXT("ACTIVE");
            case EControlBindingState::Failed:
                return TEXT("FAILED");
            case EControlBindingState::Unavailable:
            default:
                return TEXT("UNAVAILABLE");
            }
        }

        const TCHAR* SensorStateName(const EControlSensorState State)
        {
            switch (State)
            {
            case EControlSensorState::Missing:
                return TEXT("MISSING");
            case EControlSensorState::Unbound:
                return TEXT("UNBOUND");
            case EControlSensorState::Prepared:
                return TEXT("PREPARED");
            case EControlSensorState::Active:
                return TEXT("ACTIVE");
            case EControlSensorState::Failed:
                return TEXT("FAILED");
            case EControlSensorState::Unavailable:
            default:
                return TEXT("UNAVAILABLE");
            }
        }

        const TCHAR* RecordingStateName(const EControlRecordingState State)
        {
            switch (State)
            {
            case EControlRecordingState::Idle:
                return TEXT("idle");
            case EControlRecordingState::Requested:
                return TEXT("requested");
            case EControlRecordingState::Recording:
                return TEXT("recording");
            case EControlRecordingState::Committed:
                return TEXT("committed");
            case EControlRecordingState::Rejected:
                return TEXT("rejected");
            case EControlRecordingState::Failed:
                return TEXT("failed");
            case EControlRecordingState::Unavailable:
            default:
                return TEXT("unavailable");
            }
        }

        const TCHAR* StatusUIModeName(const EControlStatusUIMode Mode)
        {
            switch (Mode)
            {
            case EControlStatusUIMode::Drive:
                return TEXT("drive");
            case EControlStatusUIMode::Build:
                return TEXT("build");
            case EControlStatusUIMode::Tactical:
                return TEXT("tactical");
            case EControlStatusUIMode::Menu:
                return TEXT("menu");
            case EControlStatusUIMode::Unavailable:
            default:
                return TEXT("unavailable");
            }
        }

        const TCHAR* StatusCameraModeName(const EControlStatusCameraMode Mode)
        {
            switch (Mode)
            {
            case EControlStatusCameraMode::Follow:
                return TEXT("follow");
            case EControlStatusCameraMode::Inspection:
                return TEXT("inspection");
            case EControlStatusCameraMode::Free:
                return TEXT("free");
            case EControlStatusCameraMode::Unavailable:
            default:
                return TEXT("unavailable");
            }
        }

        TSharedRef<FJsonObject> VelocityJson(const FControlStatusVelocity& Velocity)
        {
            const TSharedRef<FJsonObject> Object = MakeShared<FJsonObject>();
            Object->SetBoolField(TEXT("available"), Velocity.bAvailable);
            Object->SetNumberField(TEXT("linear_x"), Velocity.LinearX);
            Object->SetNumberField(TEXT("linear_y"), Velocity.LinearY);
            Object->SetNumberField(TEXT("angular_z"), Velocity.AngularZ);
            return Object;
        }

        TSharedRef<FJsonObject> ReadinessFacetJson(
            const FControlStatusReadinessFacet& Facet)
        {
            const TSharedRef<FJsonObject> Object = MakeShared<FJsonObject>();
            Object->SetStringField(TEXT("state"), BindingStateName(Facet.State));
            Object->SetBoolField(TEXT("required"), Facet.bRequired);
            Object->SetStringField(TEXT("source_id"), Facet.SourceId);
            Object->SetStringField(TEXT("blocker"), Facet.Blocker);
            return Object;
        }
    }

    bool FHudScreenshotContract::ParseCommandLine(
        const FString& CommandLine,
        TArray<FHudScreenshotTarget>& OutTargets,
        bool& bOutConfigured,
        FString& OutError)
    {
        OutTargets.Reset();
        bOutConfigured = false;
        OutError.Reset();
        TArray<FString> Tokens;
        TArray<FString> Switches;
        FCommandLine::Parse(*CommandLine, Tokens, Switches);
        TArray<FString> AssignmentValueBuffer;
        if (!AssignmentValues(
                Switches,
                TEXT("LingTuHudScreenshot"),
                AssignmentValueBuffer,
                OutError))
        {
            return false;
        }
        if (AssignmentValueBuffer.Num() != 0)
        {
            OutError = TEXT("legacy_singular_hud_screenshot_argument_rejected");
            return false;
        }

        int32 TotalAssignments = 0;
        for (const FTargetDefinition& Definition : TargetDefinitions)
        {
            if (!AssignmentValues(
                    Switches,
                    Definition.CommandLineName,
                    AssignmentValueBuffer,
                    OutError))
            {
                return false;
            }
            TotalAssignments += AssignmentValueBuffer.Num();
        }
        if (TotalAssignments == 0)
        {
            return true;
        }
        if (TotalAssignments != static_cast<int32>(UE_ARRAY_COUNT(TargetDefinitions)))
        {
            OutError = TEXT("hud_screenshot_arguments_partial_or_duplicated");
            return false;
        }

        TSet<FString> UniquePaths;
        for (const FTargetDefinition& Definition : TargetDefinitions)
        {
            TArray<FString> Values;
            if (!AssignmentValues(
                    Switches,
                    Definition.CommandLineName,
                    Values,
                    OutError))
            {
                return false;
            }
            if (Values.Num() != 1)
            {
                OutError = TEXT("hud_screenshot_argument_must_appear_exactly_once");
                return false;
            }
            FString RequestedPath = Values[0];
            RequestedPath.TrimStartAndEndInline();
            if (RequestedPath.IsEmpty() || FPaths::IsRelative(RequestedPath))
            {
                OutError = TEXT("hud_screenshot_path_must_be_absolute");
                return false;
            }
            FPaths::NormalizeFilename(RequestedPath);
            FString Collapsed = RequestedPath;
            if (!FPaths::CollapseRelativeDirectories(Collapsed)
                || !FPaths::IsSamePath(Collapsed, RequestedPath))
            {
                OutError = TEXT("hud_screenshot_path_noncanonical");
                return false;
            }
            RequestedPath = CanonicalAbsolutePath(RequestedPath);
            if (FPaths::GetCleanFilename(RequestedPath) != Definition.ExpectedFilename)
            {
                OutError = TEXT("hud_screenshot_filename_mismatch");
                return false;
            }
            const FString UniqueKey = RequestedPath.ToLower();
            if (UniquePaths.Contains(UniqueKey))
            {
                OutError = TEXT("hud_screenshot_targets_must_be_distinct");
                return false;
            }
            UniquePaths.Add(UniqueKey);

            FHudScreenshotTarget Target;
            Target.CommandLineName = Definition.CommandLineName;
            Target.ScreenshotPath = MoveTemp(RequestedPath);
            Target.EvidencePath = FPaths::ChangeExtension(
                Target.ScreenshotPath,
                TEXT("evidence.json"));
            Target.ModeName = Definition.ModeName;
            Target.ExpectedFilename = Definition.ExpectedFilename;
            Target.Mode = Definition.Mode;
            OutTargets.Add(MoveTemp(Target));
        }
        bOutConfigured = true;
        return true;
    }

    bool FHudScreenshotContract::ValidateHudScreenshotTarget(
        const FHudScreenshotTarget& Target,
        const FString& ControlLogDirectory,
        const EHudScreenshotValidationPhase Phase,
        FString& OutError)
    {
        OutError.Reset();
        const FString LogDirectory = CanonicalAbsolutePath(ControlLogDirectory);
        if (FPaths::GetCleanFilename(LogDirectory) != TEXT("logs"))
        {
            OutError = TEXT("control_log_directory_invalid");
            return false;
        }
        const FString RunDirectory = CanonicalAbsolutePath(FPaths::GetPath(LogDirectory));
        const FString ExpectedDirectory = CanonicalAbsolutePath(
            FPaths::Combine(RunDirectory, TEXT("screenshots")));
        const FString ExpectedScreenshot = CanonicalAbsolutePath(
            FPaths::Combine(ExpectedDirectory, Target.ExpectedFilename));
        const FString ExpectedEvidence = FPaths::ChangeExtension(
            ExpectedScreenshot,
            TEXT("evidence.json"));
        if (!FPaths::IsSamePath(Target.ScreenshotPath, ExpectedScreenshot)
            || !FPaths::IsSamePath(Target.EvidencePath, ExpectedEvidence)
            || !FPaths::IsSamePath(FPaths::GetPath(Target.ScreenshotPath), ExpectedDirectory))
        {
            OutError = TEXT("path_outside_run_screenshots_directory");
            return false;
        }
        if (!ValidatePlainAncestorChain(Target.ScreenshotPath, OutError)
            || !ValidatePlainAncestorChain(Target.EvidencePath, OutError))
        {
            return false;
        }

        const bool bScreenshotMustExist =
            Phase != EHudScreenshotValidationPhase::BeforeRequest;
        const bool bEvidenceMustExist =
            Phase == EHudScreenshotValidationPhase::EvidenceCommitted;
        if (!ValidateFileState(
                Target.ScreenshotPath,
                bScreenshotMustExist,
                TEXT("screenshot_file_not_ready"),
                TEXT("stale_target_exists_before_request"),
                OutError)
            || !ValidateFileState(
                Target.EvidencePath,
                bEvidenceMustExist,
                TEXT("evidence_file_not_ready"),
                TEXT("stale_evidence_target_exists"),
                OutError))
        {
            return false;
        }
        return true;
    }

    bool FHudScreenshotContract::ReadPngDimensions(
        const FString& ScreenshotPath,
        int32& OutWidth,
        int32& OutHeight,
        FString& OutError)
    {
        OutWidth = 0;
        OutHeight = 0;
        OutError.Reset();
        TArray<uint8> Bytes;
        if (!FFileHelper::LoadFileToArray(Bytes, *ScreenshotPath) || Bytes.Num() < 24)
        {
            OutError = TEXT("png_header_unavailable");
            return false;
        }
        constexpr uint8 PngSignature[] = {137, 80, 78, 71, 13, 10, 26, 10};
        if (FMemory::Memcmp(Bytes.GetData(), PngSignature, UE_ARRAY_COUNT(PngSignature)) != 0
            || FMemory::Memcmp(Bytes.GetData() + 12, "IHDR", 4) != 0)
        {
            OutError = TEXT("screenshot_not_png");
            return false;
        }
        const uint32 Width = ReadBigEndianUint32(Bytes.GetData() + 16);
        const uint32 Height = ReadBigEndianUint32(Bytes.GetData() + 20);
        if (Width > static_cast<uint32>(MAX_int32)
            || Height > static_cast<uint32>(MAX_int32))
        {
            OutError = TEXT("png_dimensions_out_of_range");
            return false;
        }
        OutWidth = static_cast<int32>(Width);
        OutHeight = static_cast<int32>(Height);
        return true;
    }

    bool FHudScreenshotContract::ValidateEvidenceTempTarget(
        const FHudScreenshotTarget& Target,
        const bool bMustExist,
        FString& OutError)
    {
        OutError.Reset();
        const FString TempPath = Target.EvidencePath + TEXT(".tmp");
        if (!FPaths::IsSamePath(
                FPaths::GetPath(TempPath),
                FPaths::GetPath(Target.EvidencePath))
            || !ValidatePlainAncestorChain(TempPath, OutError))
        {
            if (OutError.IsEmpty())
            {
                OutError = TEXT("evidence_temp_path_invalid");
            }
            return false;
        }
        return ValidateFileState(
            TempPath,
            bMustExist,
            TEXT("evidence_temp_file_not_ready"),
            TEXT("stale_evidence_temp_exists"),
            OutError);
    }

    bool FHudScreenshotContract::WriteEvidenceTempFileNoFollow(
        const FHudScreenshotTarget& Target,
        const FString& EvidenceJson,
        FString& OutError)
    {
        OutError.Reset();
        if (!ValidateEvidenceTempTarget(Target, false, OutError))
        {
            return false;
        }
        const FString TempPath = Target.EvidencePath + TEXT(".tmp");
#if PLATFORM_WINDOWS
        const FTCHARToUTF8 Utf8(*EvidenceJson);
        if (Utf8.Length() < 0
            || static_cast<uint64>(Utf8.Length()) > static_cast<uint64>(MAX_uint32))
        {
            OutError = TEXT("evidence_temp_payload_too_large");
            return false;
        }
        HANDLE Handle = ::CreateFileW(
            *TempPath,
            GENERIC_WRITE | FILE_READ_ATTRIBUTES,
            FILE_SHARE_READ,
            nullptr,
            CREATE_NEW,
            FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OPEN_REPARSE_POINT,
            nullptr);
        if (Handle == INVALID_HANDLE_VALUE)
        {
            const DWORD Error = ::GetLastError();
            OutError = (Error == ERROR_FILE_EXISTS
                || Error == ERROR_ALREADY_EXISTS)
                ? TEXT("stale_evidence_temp_exists")
                : TEXT("evidence_temp_exclusive_create_failed");
            return false;
        }

        DWORD Written = 0;
        BY_HANDLE_FILE_INFORMATION Information{};
        const DWORD ByteCount = static_cast<DWORD>(Utf8.Length());
        const bool bWritten =
            ::WriteFile(Handle, Utf8.Get(), ByteCount, &Written, nullptr) != 0
            && Written == ByteCount;
        const bool bFlushed = bWritten && ::FlushFileBuffers(Handle) != 0;
        const bool bInformationRead = bFlushed
            && ::GetFileInformationByHandle(Handle, &Information) != 0;
        const bool bPlainFile = bInformationRead
            && (Information.dwFileAttributes & FILE_ATTRIBUTE_REPARSE_POINT) == 0
            && (Information.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) == 0;
        ::CloseHandle(Handle);
        if (!bPlainFile)
        {
            ::DeleteFileW(*TempPath);
            OutError = TEXT("evidence_temp_write_or_flush_failed");
            return false;
        }
#elif PLATFORM_UNIX
        const FTCHARToUTF8 TempPathUtf8(*TempPath);
        const FTCHARToUTF8 EvidenceUtf8(*EvidenceJson);
        const int Descriptor = ::open(
            TempPathUtf8.Get(),
            O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW,
            0600);
        if (Descriptor < 0)
        {
            OutError = TEXT("evidence_temp_exclusive_create_failed");
            return false;
        }
        int64 Remaining = EvidenceUtf8.Length();
        const ANSICHAR* Cursor = EvidenceUtf8.Get();
        bool bWritten = true;
        while (Remaining > 0)
        {
            const ssize_t Count = ::write(
                Descriptor,
                Cursor,
                static_cast<size_t>(Remaining));
            if (Count <= 0)
            {
                bWritten = false;
                break;
            }
            Cursor += Count;
            Remaining -= Count;
        }
        struct stat Information{};
        const bool bPlainFile = bWritten
            && ::fsync(Descriptor) == 0
            && ::fstat(Descriptor, &Information) == 0
            && S_ISREG(Information.st_mode);
        ::close(Descriptor);
        if (!bPlainFile)
        {
            ::unlink(TempPathUtf8.Get());
            OutError = TEXT("evidence_temp_write_or_flush_failed");
            return false;
        }
#else
        OutError = TEXT("evidence_temp_no_follow_write_unsupported");
        return false;
#endif
        return ValidateEvidenceTempTarget(Target, true, OutError);
    }

    bool FHudScreenshotContract::IsNextCaptureMode(
        const FHudScreenshotTarget& Target,
        const int32 NextCaptureIndex,
        const ERuntimeUIMode ActualMode)
    {
        if (NextCaptureIndex < 0
            || NextCaptureIndex >= UE_ARRAY_COUNT(TargetDefinitions))
        {
            return false;
        }
        const FTargetDefinition& Expected = TargetDefinitions[NextCaptureIndex];
        return Target.Mode == ActualMode
            && Target.Mode == Expected.Mode
            && Target.CommandLineName == Expected.CommandLineName
            && Target.ExpectedFilename == Expected.ExpectedFilename
            && Target.ModeName == Expected.ModeName;
    }

    bool FHudScreenshotContract::SerializeEvidenceJson(
        const FHudScreenshotTarget& Target,
        const FRuntimeUIStatusSnapshot& Status,
        const uint64 CapturedMonotonicNs,
        const int64 ScreenshotBytes,
        const int32 Width,
        const int32 Height,
        FString& OutJson,
        FString& OutError)
    {
        OutJson.Reset();
        OutError.Reset();
        const bool bCaptureReady =
            Target.Mode == ERuntimeUIMode::Drive
            ? Status.IsDriveCaptureReady()
            : Target.Mode == ERuntimeUIMode::Tactical
            ? Status.IsTacticalCaptureReady()
            : Status.IsMenuRecordingCaptureReady();
        if (!bCaptureReady)
        {
            OutError = TEXT("capture_status_not_qualification_ready");
            return false;
        }
        if (ScreenshotBytes <= 0 || Width != 1920 || Height != 1080)
        {
            OutError = TEXT("screenshot_media_not_exact_1920x1080");
            return false;
        }

        const FControlStatusEnvelope& FullStatus = Status.FullStatus;
        if (CapturedMonotonicNs < FullStatus.ReceivedMonotonicNs)
        {
            OutError = TEXT("captured_time_precedes_status_receive");
            return false;
        }
        const uint64 ExactStatusAgeNs =
            CapturedMonotonicNs - FullStatus.ReceivedMonotonicNs;
        const TSharedRef<FJsonObject> Root = MakeShared<FJsonObject>();
        Root->SetStringField(
            TEXT("schema"),
            TEXT("lingtu.sim.ue-hud-screenshot-evidence.v1"));
        Root->SetStringField(TEXT("state"), TEXT("CAPTURED"));
        Root->SetStringField(TEXT("capture_id"), Target.ModeName);
        Root->SetStringField(TEXT("run_id"), FullStatus.RunId);
        Root->SetStringField(TEXT("session_id"), FullStatus.SessionId);
        Root->SetStringField(TEXT("boot_id"), FullStatus.BootId);
        SetExactUint64(Root, TEXT("model_generation"), FullStatus.ModelGeneration);
        SetExactUint64(Root, TEXT("reset_generation"), FullStatus.ResetGeneration);
        SetExactUint64(Root, TEXT("captured_monotonic_ns"), CapturedMonotonicNs);
        SetExactUint64(Root, TEXT("status_age_ns"), ExactStatusAgeNs);

        const TSharedRef<FJsonObject> ControlStatus = MakeShared<FJsonObject>();
        ControlStatus->SetStringField(TEXT("status"), AckStatusName(FullStatus.Status));
        ControlStatus->SetStringField(TEXT("event_id"), FullStatus.EventId);
        ControlStatus->SetStringField(TEXT("source_id"), FullStatus.SourceId);
        SetExactUint64(ControlStatus, TEXT("source_epoch"), FullStatus.SourceEpoch);
        SetExactUint64(ControlStatus, TEXT("source_sequence"), FullStatus.SourceSequence);
        SetExactUint64(
            ControlStatus,
            TEXT("server_status_sequence"),
            FullStatus.ServerStatusSequence);
        SetExactUint64(
            ControlStatus,
            TEXT("server_monotonic_ns"),
            FullStatus.ServerMonotonicNs);
        SetExactUint64(
            ControlStatus,
            TEXT("received_monotonic_ns"),
            FullStatus.ReceivedMonotonicNs);
        ControlStatus->SetStringField(
            TEXT("ui_mode"),
            StatusUIModeName(FullStatus.UI.UIMode));
        ControlStatus->SetStringField(
            TEXT("camera_mode"),
            StatusCameraModeName(FullStatus.UI.CameraMode));
        Root->SetObjectField(TEXT("control_status"), ControlStatus);

        const TSharedRef<FJsonObject> Motion = MakeShared<FJsonObject>();
        const TSharedRef<FJsonObject> Requested = MakeShared<FJsonObject>();
        Requested->SetBoolField(
            TEXT("available"),
            FullStatus.Motion.RequestedAxes.bAvailable);
        Requested->SetNumberField(
            TEXT("forward"),
            FullStatus.Motion.RequestedAxes.Forward);
        Requested->SetNumberField(TEXT("left"), FullStatus.Motion.RequestedAxes.Left);
        Requested->SetNumberField(
            TEXT("yaw_left"),
            FullStatus.Motion.RequestedAxes.YawLeft);
        Motion->SetObjectField(TEXT("requested_axes"), Requested);
        Motion->SetObjectField(
            TEXT("admitted_twist_mps_radps"),
            VelocityJson(FullStatus.Motion.AdmittedTwist));
        Motion->SetObjectField(
            TEXT("observed_base_velocity_mps_radps"),
            VelocityJson(FullStatus.Motion.ObservedBaseVelocity));
        Root->SetObjectField(TEXT("motion"), Motion);

        const TSharedRef<FJsonObject> Readiness = MakeShared<FJsonObject>();
        Readiness->SetObjectField(
            TEXT("physics"),
            ReadinessFacetJson(FullStatus.Readiness.Physics));
        Readiness->SetObjectField(
            TEXT("control"),
            ReadinessFacetJson(FullStatus.Readiness.Control));
        Readiness->SetObjectField(
            TEXT("visual"),
            ReadinessFacetJson(FullStatus.Readiness.Visual));
        Readiness->SetObjectField(
            TEXT("sensors"),
            ReadinessFacetJson(FullStatus.Readiness.Sensors));
        Root->SetObjectField(TEXT("readiness"), Readiness);

        TArray<TSharedPtr<FJsonValue>> Sensors;
        Sensors.Reserve(FullStatus.Sensors.Num());
        for (const FControlStatusSensor& Sensor : FullStatus.Sensors)
        {
            const TSharedRef<FJsonObject> SensorObject = MakeShared<FJsonObject>();
            SensorObject->SetStringField(TEXT("stream_id"), Sensor.StreamId);
            SensorObject->SetStringField(TEXT("state"), SensorStateName(Sensor.State));
            SetExactUint64(SensorObject, TEXT("sample_count"), Sensor.SampleCount);
            SensorObject->SetStringField(TEXT("blocker"), Sensor.Blocker);
            Sensors.Add(MakeShared<FJsonValueObject>(SensorObject));
        }
        Root->SetArrayField(TEXT("sensors"), Sensors);

        const TSharedRef<FJsonObject> Recording = MakeShared<FJsonObject>();
        Recording->SetStringField(
            TEXT("state"),
            RecordingStateName(FullStatus.Recording.State));
        SetExactUint64(
            Recording,
            TEXT("elapsed_sim_time_ns"),
            FullStatus.Recording.ElapsedSimTimeNs);
        Recording->SetStringField(TEXT("artifact_id"), FullStatus.Recording.ArtifactId);
        Recording->SetStringField(TEXT("blocker"), FullStatus.Recording.Blocker);
        Root->SetObjectField(TEXT("recording"), Recording);

        const TSharedRef<FJsonObject> Screenshot = MakeShared<FJsonObject>();
        Screenshot->SetStringField(
            TEXT("basename"),
            FPaths::GetCleanFilename(Target.ScreenshotPath));
        SetExactInt64(Screenshot, TEXT("bytes"), ScreenshotBytes);
        Screenshot->SetNumberField(TEXT("width"), Width);
        Screenshot->SetNumberField(TEXT("height"), Height);
        Screenshot->SetBoolField(TEXT("show_ui"), true);
        Root->SetObjectField(TEXT("screenshot"), Screenshot);
        Root->SetBoolField(TEXT("qualification_ready"), true);

        const TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&OutJson);
        if (!FJsonSerializer::Serialize(Root, Writer))
        {
            OutJson.Reset();
            OutError = TEXT("evidence_json_serialize_failed");
            return false;
        }
        OutJson.AppendChar(TEXT('\n'));
        return true;
    }
}
