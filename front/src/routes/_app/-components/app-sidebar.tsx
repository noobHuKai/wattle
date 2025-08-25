import * as React from "react";
import {
	IconChartBar,
	IconDashboard,
	IconHelp,
	IconInnerShadowTop,
	IconListDetails,
	IconSearch,
	IconSettings,
} from "@tabler/icons-react";
import { Link } from "@tanstack/react-router";

import { NavMain } from "./nav-main";
import { NavSecondary } from "./nav-secondary";
import { NavUser } from "./nav-user";
import {
	Sidebar,
	SidebarContent,
	SidebarFooter,
	SidebarHeader,
	SidebarMenu,
	SidebarMenuButton,
	SidebarMenuItem,
} from "@/components/ui/sidebar";

const data = {
	user: {
		name: "shadcn",
		email: "m@example.com",
		avatar: "/avatars/shadcn.jpg",
	},
	navMain: [
		{
			title: "仪表盘",
			url: "/dashboard",
			icon: IconDashboard,
		},
		// {
		// 	title: "任务",
		// 	url: "/task",
		// 	icon: IconListDetails,
		// },
		{
			title: "任务组",
			url: "/task_group",
			icon: IconChartBar,
		},
		{
			title: "工作流",
			url: "/workflow",
			icon: IconChartBar,
		},
		{
			title: "仿真",
			url: "/sim",
			icon: IconChartBar,
		},
	],
	navSecondary: [
		{
			title: "Settings",
			url: "/settings",
			icon: IconSettings,
		},
		{
			title: "Get Help",
			url: "/help",
			icon: IconHelp,
		},
		{
			title: "Search",
			url: "/search",
			icon: IconSearch,
		},
	],
};

export function AppSidebar({ ...props }: React.ComponentProps<typeof Sidebar>) {
	return (
		<Sidebar collapsible="offcanvas" {...props}>
			<SidebarHeader>
				<SidebarMenu>
					<SidebarMenuItem>
						<SidebarMenuButton
							asChild
							className="data-[slot=sidebar-menu-button]:!p-1.5"
						>
							<Link to="/dashboard">
								<IconInnerShadowTop className="!size-5" />
								<span className="text-base font-semibold">Wattle</span>
							</Link>
						</SidebarMenuButton>
					</SidebarMenuItem>
				</SidebarMenu>
			</SidebarHeader>
			<SidebarContent>
				<NavMain items={data.navMain} />
				<NavSecondary items={data.navSecondary} className="mt-auto" />
			</SidebarContent>
			<SidebarFooter>
				<NavUser user={data.user} />
			</SidebarFooter>
		</Sidebar>
	);
}
