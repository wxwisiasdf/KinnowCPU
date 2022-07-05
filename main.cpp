// STL
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <exception>

// Verilator
#include <verilated.h>
#include <verilated_vpi.h>

// SDL
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

// Project
#include "Vlimn2600_System.h"

#define SCREEN_WIDTH 1280
#define SCREEN_HEIGHT 640
#define ARRAY_LENGTH(x) (sizeof(x) / sizeof((x)[0]))

int main(int argc, char **argv, char **env)
{
    SDL_Color colors[256];
    for(size_t i = 0; i < 256; i++)
    {
        colors[i].r = colors[i].g = colors[i].b = (Uint8)i;
    }

    auto *window = SDL_CreateWindow("Limn2600", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    if (!window)
        throw std::runtime_error(std::string("Window creation failed: ") + SDL_GetError());

    auto *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!renderer)
        throw std::runtime_error(std::string("Renderer creation failed: ") + SDL_GetError());

    // Create the system
    auto top = std::unique_ptr<Vlimn2600_System>(new Vlimn2600_System());

    Verilated::debug(0);
    Verilated::randReset(2);
    Verilated::traceEverOn(true);
    Verilated::commandArgs(argc, argv);
    Verilated::mkdir("logs");

    // ROM
    bool show_rom = true; // Whetever to show the ROM
    auto *rom_texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_YUY2, SDL_TEXTUREACCESS_TARGET, 128, 128);
    if (!rom_texture)
        throw std::runtime_error(std::string("ROM Texture creation failed: ") + SDL_GetError());

    // RAM
    bool show_ram = true; // Whetever to show the RAM
    auto *ram_texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_YUY2, SDL_TEXTUREACCESS_TARGET, 256, 256);
    if (!ram_texture)
        throw std::runtime_error(std::string("RAM Texture creation failed: ") + SDL_GetError());

    vluint64_t main_time = 0;
    bool run = true;
    while (!Verilated::gotFinish() && run)
    {
        printf("perf: Begin tick #%llu\n", main_time);
        top->rst = (main_time < 10) ? 1 : 0;
        top->clk = !top->clk;
        main_time++;
#if VM_COVERAGE
        if (main_time < 5)
        {
            // Zero coverage if still early in reset, otherwise toggles there may
            // falsely indicate a signal is covered
            VerilatedCov::zero();
        }
#endif
        top->eval();

        SDL_Event e;
        if (SDL_PollEvent(&e))
        {
            switch (e.type)
            {
            case SDL_QUIT:
                run = false;
                break;
            case SDL_KEYUP:
                switch(e.key.keysym.sym)
                {
                    case SDLK_F1:
                        show_ram = !show_ram;
                        break;
                    case SDLK_F2:
                        show_rom = !show_rom;
                        break;
                    default:
                        break;
                }
                break;
            default:
                break;
            }
        }

        SDL_RenderClear(renderer);

        SDL_Rect dstrect;
        int x_offset = 0;
        if (show_rom)
        {
            dstrect.x = x_offset;
            dstrect.y = 0;
            int w, h;
            SDL_QueryTexture(rom_texture, NULL, NULL, &w, &h);
            dstrect.w = w * 2;
            dstrect.h = h * 2;
            SDL_UpdateTexture(rom_texture, NULL, &top->limn2600_System__DOT__SRAM__DOT__rom, w * sizeof(IData));
            SDL_RenderCopy(renderer, rom_texture, NULL, &dstrect);
            x_offset += dstrect.w;
        }
        if (show_ram)
        {
            dstrect.x = x_offset;
            dstrect.y = 0;
            int w, h;
            SDL_QueryTexture(rom_texture, NULL, NULL, &w, &h);
            dstrect.w = w * 4;
            dstrect.h = h * 4;
            SDL_UpdateTexture(ram_texture, NULL, &top->limn2600_System__DOT__SRAM__DOT__ram, w * sizeof(IData));
            SDL_RenderCopy(renderer, ram_texture, NULL, &dstrect);
            x_offset += dstrect.w;
        }

        SDL_RenderPresent(renderer);
        printf("perf: End tick\n");
    }
    top->final();

    //  Coverage analysis (since test passed)
#if VM_COVERAGE
    Verilated::mkdir("logs");
    VerilatedCov::write("logs/coverage.dat");
#endif
    SDL_DestroyTexture(rom_texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
