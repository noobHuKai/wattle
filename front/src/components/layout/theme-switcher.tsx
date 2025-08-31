"use client"

import { useState } from "react"
import { Button } from "@/components/ui/button"
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuSeparator,
  DropdownMenuTrigger,
  DropdownMenuSub,
  DropdownMenuSubContent,
  DropdownMenuSubTrigger,
} from "@/components/ui/dropdown-menu"
import { Moon, Sun, Palette, Check } from "lucide-react"
import { useTheme } from "next-themes"

const themes = [
  { name: "Default", value: "default" },
  { name: "New York", value: "new-york" },
  { name: "Blue", value: "blue" },
  { name: "Green", value: "green" },
  { name: "Orange", value: "orange" },
  { name: "Red", value: "red" },
  { name: "Rose", value: "rose" },
  { name: "Violet", value: "violet" },
]

const colors = [
  { name: "Slate", value: "slate", color: "bg-slate-500" },
  { name: "Gray", value: "gray", color: "bg-gray-500" },
  { name: "Zinc", value: "zinc", color: "bg-zinc-500" },
  { name: "Neutral", value: "neutral", color: "bg-neutral-500" },
  { name: "Stone", value: "stone", color: "bg-stone-500" },
  { name: "Red", value: "red", color: "bg-red-500" },
  { name: "Orange", value: "orange", color: "bg-orange-500" },
  { name: "Amber", value: "amber", color: "bg-amber-500" },
  { name: "Yellow", value: "yellow", color: "bg-yellow-500" },
  { name: "Lime", value: "lime", color: "bg-lime-500" },
  { name: "Green", value: "green", color: "bg-green-500" },
  { name: "Emerald", value: "emerald", color: "bg-emerald-500" },
  { name: "Teal", value: "teal", color: "bg-teal-500" },
  { name: "Cyan", value: "cyan", color: "bg-cyan-500" },
  { name: "Sky", value: "sky", color: "bg-sky-500" },
  { name: "Blue", value: "blue", color: "bg-blue-500" },
  { name: "Indigo", value: "indigo", color: "bg-indigo-500" },
  { name: "Violet", value: "violet", color: "bg-violet-500" },
  { name: "Purple", value: "purple", color: "bg-purple-500" },
  { name: "Fuchsia", value: "fuchsia", color: "bg-fuchsia-500" },
  { name: "Pink", value: "pink", color: "bg-pink-500" },
  { name: "Rose", value: "rose", color: "bg-rose-500" },
]

export function ThemeSwitcher() {
  const { theme, setTheme } = useTheme()
  const [currentTheme, setCurrentTheme] = useState("default")
  const [currentColor, setCurrentColor] = useState("blue")

  const handleThemeChange = (newTheme: string) => {
    setCurrentTheme(newTheme)
    // Apply theme to document root
    document.documentElement.setAttribute("data-theme", newTheme)
  }

  const handleColorChange = (newColor: string) => {
    setCurrentColor(newColor)
    // Apply color to document root
    document.documentElement.setAttribute("data-color", newColor)
  }

  return (
    <DropdownMenu>
      <DropdownMenuTrigger asChild>
        <Button variant="ghost" size="icon" className="h-9 w-9">
          <Sun className="h-4 w-4 rotate-0 scale-100 transition-all dark:-rotate-90 dark:scale-0" />
          <Moon className="absolute h-4 w-4 rotate-90 scale-0 transition-all dark:rotate-0 dark:scale-100" />
          <span className="sr-only">Toggle theme</span>
        </Button>
      </DropdownMenuTrigger>
      <DropdownMenuContent align="end" className="w-56">
        <DropdownMenuItem onClick={() => setTheme("light")}>
          <Sun className="mr-2 h-4 w-4" />
          Light
        </DropdownMenuItem>
        <DropdownMenuItem onClick={() => setTheme("dark")}>
          <Moon className="mr-2 h-4 w-4" />
          Dark
        </DropdownMenuItem>
        <DropdownMenuItem onClick={() => setTheme("system")}>
          <Palette className="mr-2 h-4 w-4" />
          System
        </DropdownMenuItem>
        <DropdownMenuSeparator />
        <DropdownMenuSub>
          <DropdownMenuSubTrigger>
            <Palette className="mr-2 h-4 w-4" />
            Themes
          </DropdownMenuSubTrigger>
          <DropdownMenuSubContent>
            {themes.map((themeOption) => (
              <DropdownMenuItem key={themeOption.value} onClick={() => handleThemeChange(themeOption.value)}>
                {currentTheme === themeOption.value && <Check className="mr-2 h-4 w-4" />}
                {currentTheme !== themeOption.value && <div className="mr-6" />}
                {themeOption.name}
              </DropdownMenuItem>
            ))}
          </DropdownMenuSubContent>
        </DropdownMenuSub>
        <DropdownMenuSub>
          <DropdownMenuSubTrigger>
            <div className={`mr-2 h-4 w-4 rounded-full ${colors.find((c) => c.value === currentColor)?.color}`} />
            Colors
          </DropdownMenuSubTrigger>
          <DropdownMenuSubContent className="w-48">
            <div className="grid grid-cols-6 gap-1 p-2">
              {colors.map((colorOption) => (
                <button
                  key={colorOption.value}
                  onClick={() => handleColorChange(colorOption.value)}
                  className={`h-6 w-6 rounded-full ${colorOption.color} hover:scale-110 transition-transform ${
                    currentColor === colorOption.value ? "ring-2 ring-foreground ring-offset-2" : ""
                  }`}
                  title={colorOption.name}
                />
              ))}
            </div>
          </DropdownMenuSubContent>
        </DropdownMenuSub>
      </DropdownMenuContent>
    </DropdownMenu>
  )
}
