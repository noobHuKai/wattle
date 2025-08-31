import { createFileRoute, Outlet, useLocation } from "@tanstack/react-router";

import { AppSidebar } from "./_app/-components/app-sidebar";
import { SiteHeader } from "@/components/site-header";
import { SidebarInset, SidebarProvider } from "@/components/ui/sidebar";
import { Toaster } from "@/components/ui/sonner";

// Control Flow components
import { ThemeProvider } from '@/components/theme-provider'
import { Topbar } from '@/components/layout/topbar'
import { Sidebar } from '@/components/layout/sidebar'
import { Suspense } from 'react'

export const Route = createFileRoute("/_app")({
	component: App,
});

function App() {
	const location = useLocation()
	const isControlFlow = ['dashboard', 'workflows', 'nodes'].some(path => 
		location.pathname.includes(`/${path}`)
	)

	if (isControlFlow) {
		return (
			<ThemeProvider attribute="class" defaultTheme="system" enableSystem disableTransitionOnChange>
				<div className="h-screen flex flex-col">
					<Topbar />
					<div className="flex-1 flex overflow-hidden">
						<Sidebar />
						<main className="flex-1 overflow-y-auto bg-background">
							<div className="p-6">
								<Suspense fallback={<div>Loading...</div>}>
									<Outlet />
								</Suspense>
							</div>
						</main>
					</div>
				</div>
				<Toaster />
			</ThemeProvider>
		);
	}

	return (
		<SidebarProvider
			style={
				{
					"--sidebar-width": "calc(var(--spacing) * 72)",
					"--header-height": "calc(var(--spacing) * 12)",
				} as React.CSSProperties
			}
		>
			<AppSidebar variant="inset" />
			<SidebarInset>
				<SiteHeader />
				<div className="flex flex-1 flex-col">
					<div className="@container/main flex flex-1 flex-col gap-2">
						<div className="flex flex-col gap-4 py-4 md:gap-6 md:py-6">
							<Outlet />
						</div>
					</div>
				</div>
			</SidebarInset>
			<Toaster />
		</SidebarProvider>
	);
}
