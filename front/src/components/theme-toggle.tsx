import { IconMoon, IconSun, IconDeviceDesktop } from "@tabler/icons-react";
import { Button } from "@/components/ui/button";
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuLabel,
  DropdownMenuRadioGroup,
  DropdownMenuRadioItem,
  DropdownMenuSeparator,
  DropdownMenuTrigger,
} from "@/components/ui/dropdown-menu";
import { useTheme, type ThemeMode, type ThemeColor } from "@/hooks/theme-context";

const themeColors: Array<{ value: ThemeColor; label: string; colorClass: string }> = [
  { value: "default", label: "Default", colorClass: "bg-zinc-900 dark:bg-zinc-100" },
  { value: "red", label: "Red", colorClass: "bg-red-500" },
  { value: "rose", label: "Rose", colorClass: "bg-rose-500" },
  { value: "orange", label: "Orange", colorClass: "bg-orange-500" },
  { value: "green", label: "Green", colorClass: "bg-green-500" },
  { value: "blue", label: "Blue", colorClass: "bg-blue-500" },
  { value: "yellow", label: "Yellow", colorClass: "bg-yellow-500" },
  { value: "violet", label: "Violet", colorClass: "bg-violet-500" },
];

export function ThemeToggle() {
  const { mode, color, setMode, setColor } = useTheme();

  return (
    <DropdownMenu>
      <DropdownMenuTrigger asChild>
        <Button variant="ghost" size="sm" className="h-8 w-8 px-0">
          <IconSun className="h-[1.2rem] w-[1.2rem] rotate-0 scale-100 transition-all dark:-rotate-90 dark:scale-0" />
          <IconMoon className="absolute h-[1.2rem] w-[1.2rem] rotate-90 scale-0 transition-all dark:rotate-0 dark:scale-100" />
          <span className="sr-only">Toggle theme</span>
        </Button>
      </DropdownMenuTrigger>
      <DropdownMenuContent align="end" className="w-56">
        <DropdownMenuLabel>主题模式</DropdownMenuLabel>
        <DropdownMenuRadioGroup value={mode} onValueChange={(value) => setMode(value as ThemeMode)}>
          <DropdownMenuRadioItem value="light" className="flex items-center gap-2">
            <IconSun className="h-4 w-4" />
            明亮
          </DropdownMenuRadioItem>
          <DropdownMenuRadioItem value="dark" className="flex items-center gap-2">
            <IconMoon className="h-4 w-4" />
            深色
          </DropdownMenuRadioItem>
          <DropdownMenuRadioItem value="system" className="flex items-center gap-2">
            <IconDeviceDesktop className="h-4 w-4" />
            跟随系统
          </DropdownMenuRadioItem>
        </DropdownMenuRadioGroup>
        
        <DropdownMenuSeparator />
        
        <DropdownMenuLabel>主题色</DropdownMenuLabel>
        <div className="grid grid-cols-4 gap-2 p-2">
          {themeColors.map((themeColor) => (
            <button
              key={themeColor.value}
              onClick={() => setColor(themeColor.value)}
              className={`
                group relative h-8 w-8 rounded-md border-2 transition-all hover:scale-105
                ${color === themeColor.value 
                  ? "border-foreground shadow-md" 
                  : "border-border hover:border-foreground/50"
                }
              `}
              title={themeColor.label}
            >
              <div className={`h-full w-full rounded-sm ${themeColor.colorClass}`} />
              {color === themeColor.value && (
                <div className="absolute inset-0 flex items-center justify-center">
                  <div className="h-1 w-1 rounded-full bg-white dark:bg-black" />
                </div>
              )}
            </button>
          ))}
        </div>
      </DropdownMenuContent>
    </DropdownMenu>
  );
}
