/* Copy Assetto Corsa shared-memory pages into the Proton prefix.
 * Run inside that prefix while a session is on track. Linux reads
 * C:\uvis\physics.bin and C:\uvis\graphics.bin. */
#include <windows.h>
#include <stdio.h>
#include <string.h>

#define PHYSICS_BYTES 580
#define GRAPHICS_BYTES 264

#define CONTROLS_BYTES 72

static HANDLE controls_map;
static unsigned char *controls_view;

static void ensure_controls(void) {
    if (controls_view) {
        return;
    }
    controls_map = CreateFileMappingW(
        INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, CONTROLS_BYTES,
        L"AcTools.CSP.NewBehaviour.CustomAI.CarControls0.v0");
    if (!controls_map) {
        return;
    }
    controls_view = MapViewOfFile(controls_map, FILE_MAP_ALL_ACCESS, 0, 0, CONTROLS_BYTES);
    if (controls_view) {
        ZeroMemory(controls_view, CONTROLS_BYTES);
    }
}

/* drive.bin is three floats: gas, brake, steer. CSP reads this page at 333 Hz
 * and replaces the player controls once the track allows custom AI. */
static void write_controls(void) {
    HANDLE file;
    float pedal[3];
    DWORD got = 0;
    ensure_controls();
    if (!controls_view) {
        return;
    }
    file = CreateFileW(L"C:\\uvis\\drive.bin", GENERIC_READ, FILE_SHARE_READ, NULL,
                        OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (file == INVALID_HANDLE_VALUE) {
        return;
    }
    if (!ReadFile(file, pedal, sizeof(pedal), &got, NULL) || got < sizeof(pedal)) {
        CloseHandle(file);
        return;
    }
    CloseHandle(file);
    memcpy(controls_view + 0, &pedal[0], 4);
    memcpy(controls_view + 4, &pedal[1], 4);
    *(float *)(controls_view + 8) = 1.0f; /* clutch engaged */
    memcpy(controls_view + 12, &pedal[2], 4);
    controls_view[68] = 1; /* autoshift */
}

static void write_page(const wchar_t *mapping, const wchar_t *path, DWORD size) {
    HANDLE mapped = OpenFileMappingW(FILE_MAP_READ, FALSE, mapping);
    HANDLE file;
    void *view;
    DWORD wrote = 0;
    if (!mapped) {
        return;
    }
    view = MapViewOfFile(mapped, FILE_MAP_READ, 0, 0, size);
    if (!view) {
        CloseHandle(mapped);
        return;
    }
    /* Write a side file, then rename it. CREATE_ALWAYS on the live
     * file empties it for a moment and the reader sees 0 bytes. */
    wchar_t tmp[MAX_PATH];
    _snwprintf(tmp, MAX_PATH, L"%s.tmp", path);
    file = CreateFileW(tmp, GENERIC_WRITE, 0, NULL,
                        CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
    if (file != INVALID_HANDLE_VALUE) {
        WriteFile(file, view, size, &wrote, NULL);
        CloseHandle(file);
        if (wrote == size) {
            MoveFileExW(tmp, path, MOVEFILE_REPLACE_EXISTING);
        }
    }
    UnmapViewOfFile(view);
    CloseHandle(mapped);
}

int main(void) {
    CreateDirectoryW(L"C:\\uvis", NULL);
    SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOOPENFILEERRORBOX | SEM_NOGPFAULTERRORBOX);
    for (;;) {
        write_page(L"Local\\acpmf_physics", L"C:\\uvis\\physics.bin", PHYSICS_BYTES);
        write_page(L"Local\\acpmf_graphics", L"C:\\uvis\\graphics.bin", GRAPHICS_BYTES);
        write_controls();
        Sleep(20);
    }
    return 0;
}
