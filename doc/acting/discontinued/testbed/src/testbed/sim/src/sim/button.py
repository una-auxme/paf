import pygame


class Button:
    width = 60
    height = 40

    def __init__(self, pos_x: int, pos_y: int, label: str):
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.label = label

    def check_collision(self, mouse_x: int, mouse_y: int) -> bool:
        return (
            self.pos_x <= mouse_x <= self.pos_x + self.width
            and self.pos_y <= mouse_y <= self.pos_y + self.height
        )

    def draw(self, screen):

        pygame.draw.rect(
            screen,
            (200, 200, 200),
            (self.pos_x, self.pos_y, self.width, self.height),
        )
        text = pygame.font.Font(None, 36).render(self.label, True, (0, 0, 0))
        screen.blit(
            text,
            (
                self.pos_x + (self.width - text.get_width()) // 2,
                self.pos_y + (self.height - text.get_height()) // 2,
            ),
        )
