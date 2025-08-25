import { type Icon } from "@tabler/icons-react";
import { useLocation } from "@tanstack/react-router";

import {
	SidebarGroup,
	SidebarGroupContent,
	SidebarMenu,
	SidebarMenuButton,
	SidebarMenuItem,
} from "@/components/ui/sidebar";
import { Link } from "@tanstack/react-router";

export function NavMain({
	items,
}: {
	items: {
		title: string;
		url: string;
		icon?: Icon;
	}[];
}) {
	const location = useLocation();
	
	return (
		<SidebarGroup>
			<SidebarGroupContent className="flex flex-col gap-2">
				{/* <SidebarMenu>
					<SidebarMenuItem className="flex items-center gap-2">
						<SidebarMenuButton
							tooltip="Quick Create"
							className="bg-primary text-primary-foreground hover:bg-primary/90 hover:text-primary-foreground active:bg-primary/90 active:text-primary-foreground min-w-8 duration-200 ease-linear"
						>
							<IconCirclePlusFilled />
							<span>Quick Create</span>
						</SidebarMenuButton>
					</SidebarMenuItem>
				</SidebarMenu> */}
				<SidebarMenu>
					{items.map((item) => {
						const isActive = location.pathname === item.url || location.pathname.startsWith(`${item.url}/`);
						
						return (
							<Link to={item.url} key={item.title}>
								<SidebarMenuItem>
									<SidebarMenuButton 
										tooltip={item.title}
										isActive={isActive}
									>
										{item.icon && <item.icon />}
										<span>{item.title}</span>
									</SidebarMenuButton>
								</SidebarMenuItem>
							</Link>
						);
					})}
				</SidebarMenu>
			</SidebarGroupContent>
		</SidebarGroup>
	);
}
