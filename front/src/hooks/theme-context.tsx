import React, { createContext, useContext, useEffect, useState } from "react";

// 主题模式类型
export type ThemeMode = "light" | "dark" | "system";

// 主题色类型
export type ThemeColor = 
  | "default" 
  | "red" 
  | "rose" 
  | "orange" 
  | "green" 
  | "blue" 
  | "yellow" 
  | "violet";

// 主题上下文类型
interface ThemeContextType {
  mode: ThemeMode;
  color: ThemeColor;
  setMode: (mode: ThemeMode) => void;
  setColor: (color: ThemeColor) => void;
  resolvedTheme: "light" | "dark";
}

const ThemeContext = createContext<ThemeContextType | undefined>(undefined);

// 主题色配置
const themeColors: Record<ThemeColor, { light: Record<string, string>; dark: Record<string, string> }> = {
  default: {
    light: {
      "--primary": "oklch(0.205 0 0)",
      "--primary-foreground": "oklch(0.985 0 0)",
    },
    dark: {
      "--primary": "oklch(0.922 0 0)",
      "--primary-foreground": "oklch(0.205 0 0)",
    },
  },
  red: {
    light: {
      "--primary": "oklch(0.637 0.237 25.331)",
      "--primary-foreground": "oklch(0.985 0.013 17.38)",
    },
    dark: {
      "--primary": "oklch(0.704 0.191 22.216)",
      "--primary-foreground": "oklch(0.985 0.013 17.38)",
    },
  },
  rose: {
    light: {
      "--primary": "oklch(0.632 0.204 353.837)",
      "--primary-foreground": "oklch(0.985 0.007 357.38)",
    },
    dark: {
      "--primary": "oklch(0.693 0.162 352.65)",
      "--primary-foreground": "oklch(0.985 0.007 357.38)",
    },
  },
  orange: {
    light: {
      "--primary": "oklch(0.679 0.182 70.67)",
      "--primary-foreground": "oklch(0.985 0.024 106.89)",
    },
    dark: {
      "--primary": "oklch(0.743 0.145 68.63)",
      "--primary-foreground": "oklch(0.985 0.024 106.89)",
    },
  },
  green: {
    light: {
      "--primary": "oklch(0.546 0.137 155.233)",
      "--primary-foreground": "oklch(0.985 0.017 155.58)",
    },
    dark: {
      "--primary": "oklch(0.623 0.109 154.449)",
      "--primary-foreground": "oklch(0.985 0.017 155.58)",
    },
  },
  blue: {
    light: {
      "--primary": "oklch(0.508 0.207 255.5)",
      "--primary-foreground": "oklch(0.985 0.011 264.33)",
    },
    dark: {
      "--primary": "oklch(0.588 0.165 254.7)",
      "--primary-foreground": "oklch(0.985 0.011 264.33)",
    },
  },
  yellow: {
    light: {
      "--primary": "oklch(0.714 0.191 93.333)",
      "--primary-foreground": "oklch(0.985 0.024 106.89)",
    },
    dark: {
      "--primary": "oklch(0.769 0.153 91.96)",
      "--primary-foreground": "oklch(0.985 0.024 106.89)",
    },
  },
  violet: {
    light: {
      "--primary": "oklch(0.548 0.207 302.71)",
      "--primary-foreground": "oklch(0.985 0.013 320.43)",
    },
    dark: {
      "--primary": "oklch(0.627 0.165 301.9)",
      "--primary-foreground": "oklch(0.985 0.013 320.43)",
    },
  },
};

export function ThemeProvider({ 
  children, 
  defaultMode = "system", 
  defaultColor = "default" 
}: {
  children: React.ReactNode;
  defaultMode?: ThemeMode;
  defaultColor?: ThemeColor;
}) {
  const [mode, setModeState] = useState<ThemeMode>(() => {
    if (typeof window !== "undefined") {
      const saved = localStorage.getItem("theme-mode");
      return (saved as ThemeMode) || defaultMode;
    }
    return defaultMode;
  });

  const [color, setColorState] = useState<ThemeColor>(() => {
    if (typeof window !== "undefined") {
      const saved = localStorage.getItem("theme-color");
      return (saved as ThemeColor) || defaultColor;
    }
    return defaultColor;
  });

  const [resolvedTheme, setResolvedTheme] = useState<"light" | "dark">("light");

  // 设置主题模式
  const setMode = (newMode: ThemeMode) => {
    setModeState(newMode);
    if (typeof window !== "undefined") {
      localStorage.setItem("theme-mode", newMode);
    }
  };

  // 设置主题色
  const setColor = (newColor: ThemeColor) => {
    setColorState(newColor);
    if (typeof window !== "undefined") {
      localStorage.setItem("theme-color", newColor);
    }
  };

  // 应用主题到DOM
  useEffect(() => {
    const root = document.documentElement;
    
    // 确定实际主题
    let actualTheme: "light" | "dark" = "light";
    
    if (mode === "system") {
      const mediaQuery = window.matchMedia("(prefers-color-scheme: dark)");
      actualTheme = mediaQuery.matches ? "dark" : "light";
      
      const handleChange = (e: MediaQueryListEvent) => {
        const newTheme = e.matches ? "dark" : "light";
        setResolvedTheme(newTheme);
        applyTheme(newTheme);
      };
      
      mediaQuery.addEventListener("change", handleChange);
      
      // 清理函数
      return () => mediaQuery.removeEventListener("change", handleChange);
    } else {
      actualTheme = mode;
    }
    
    setResolvedTheme(actualTheme);
    applyTheme(actualTheme);
    
    function applyTheme(theme: "light" | "dark") {
      // 移除旧的主题类
      root.classList.remove("light", "dark");
      // 添加新的主题类
      root.classList.add(theme);
      
      // 应用主题色变量
      const colorVars = themeColors[color][theme];
      Object.entries(colorVars).forEach(([key, value]) => {
        root.style.setProperty(key, value);
      });
    }
  }, [mode, color]);

  return (
    <ThemeContext.Provider 
      value={{ 
        mode, 
        color, 
        setMode, 
        setColor, 
        resolvedTheme 
      }}
    >
      {children}
    </ThemeContext.Provider>
  );
}

export function useTheme() {
  const context = useContext(ThemeContext);
  if (context === undefined) {
    throw new Error("useTheme must be used within a ThemeProvider");
  }
  return context;
}