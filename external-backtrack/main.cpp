#include "stdafx.hpp"
#include <thread>
#include <process.h> 

std::unique_ptr<c_memory> g_ptr_memory				= std::make_unique<c_memory>();
std::unique_ptr<c_backtrack> g_ptr_backtrack		= std::make_unique<c_backtrack>();
c_module* engine_module = nullptr;
c_module* client_module = nullptr;
bool end = false;
HHOOK mouse_hook;
bool mouse_down = false;

unsigned int __stdcall backtrack() {
	for (;; ) {
		if (mouse_down) {
			g_ptr_backtrack->do_backtrack();
		}
	}
}

unsigned int __stdcall update_data() {
	for (;; ) {
		if (end)
			break;

		if (g_ptr_backtrack->is_ingame())
			g_ptr_backtrack->update();

		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	return 0;
}

unsigned int __stdcall other() {
	for (;; ) {
		if (end)
			break;

		if (g_ptr_backtrack->is_ingame()) {
			g_ptr_backtrack->set_localplr();
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	return 0;
}

LRESULT __stdcall hk_mouse(int nCode, WPARAM wParam, LPARAM lParam)
{
	if (nCode >= 0)
	{
		switch (wParam) {
		case WM_LBUTTONDOWN:
		{
			if (GetForegroundWindow() == g_ptr_memory->get_window() && g_ptr_backtrack->is_ingame()) {
				g_ptr_backtrack->do_backtrack();
				mouse_down = true;
			}

			break;
		}
		case WM_LBUTTONUP:
		{
			mouse_down = false;
			break;
		}
		default:
		{
			break;
		}
		}
	}

	return CallNextHookEx(mouse_hook, nCode, wParam, lParam);
}

void unhook_mouse()
{
	UnhookWindowsHookEx(mouse_hook);
}

int main() {
	printf_s("\t\t\t\t\t\texternal-backtrack\n\n");

	printf_s("Waiting for Counter-Strike: Global Offensive\n");

	HWND game_window = nullptr;
	while (!game_window) {
		game_window = FindWindowA(nullptr, "Counter-Strike: Global Offensive");
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(5000));
	printf_s("Counter-Strike: Global Offensive found\n");

	if (!g_ptr_memory->attach_process("csgo.exe")) {
		printf_s("can not attack to csgo.exe... exiting now\n");
		return -1;
	}

	while (!engine_module || !client_module) {
		engine_module = g_ptr_memory->get_module("engine.dll");
		client_module = g_ptr_memory->get_module("client_panorama.dll");

		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}

	g_ptr_memory->set_process_window(game_window);

	if (engine_module && client_module)
		printf_s("engine and client module found\n");
	else {
		printf_s("can not find engine and client module... exiting now\n");
		return -1;
	}

	offsets::dw_clientstate = g_ptr_memory->read_memory<ptrdiff_t>(engine_module->get_image_base() + offsets::dw_clientstate);

	std::thread t1(backtrack);
	std::thread t2(update_data);
	std::thread t3(other);

	t1.detach();
	t2.detach();
	t3.detach();


	mouse_hook = SetWindowsHookEx(WH_MOUSE_LL, hk_mouse, nullptr, 0);
	if (!mouse_hook) {
		printf_s("Failed to set MouseHook %d \n", GetLastError());
	}
	printf_s("MouseHook set\n\n");
	if (RegisterHotKey(
		nullptr,
		1,
		MOD_NOREPEAT,
		VK_F6))
	{
		printf_s("press F6 to exit safely\n");
	}

	MSG msg;
	while (GetMessage(&msg, nullptr, 0, 0))
	{
		if (msg.message == WM_HOTKEY)
		{
			if (msg.wParam == 1)
				end = true;
		}

		TranslateMessage(&msg);
		DispatchMessage(&msg);

		if (end) {
			unhook_mouse();
			break;
		}
	}

	g_ptr_backtrack->send_packet(true);

	g_ptr_memory->detach_process();
	delete engine_module;
	delete client_module;

	return 0;
}
