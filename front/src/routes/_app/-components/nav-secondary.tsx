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

export function NavSecondary({
	items,
	...props
}: {
	items: {
		title: string;
		url: string;
		icon: Icon;
	}[];
} & React.ComponentPropsWithoutRef<typeof SidebarGroup>) {
	const location = useLocation();
	
	return (
		<SidebarGroup {...props}>
			<SidebarGroupContent>
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
										<item.icon />
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
